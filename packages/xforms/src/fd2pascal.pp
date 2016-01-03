Program fd2pascal;

{ ---------------------------------------------------------------------------
    Program to convert forms fdesign file to pascal code
    Copyright (C) 1997  Michael Van Canneyt

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 1, or (at your option)
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
  --------------------------------------------------------------------------- }


uses
  baseunix,
  Unix,
  unixutil;

Const RevString = '$Revision: 1.5 $';
  NrOptions = 4;
  Options   : Array[0..NrOptions] Of String[20] =
              ('v','callback','main','altformat','compensate');

Type
  { Properties of an object }
  ContProps=(CPclass,CPtype,CPbox,CPBoxtype,CPColors,CPalignment,CPstyle,CPsize,
             CPlcol,CPlabel,CPShortcut,CPresize,CPgravity,CPname,CPCallback,
             CPargument,
             CPinvalid);
  { Properties of an object for which defaults must be set }
  AdjProps=(APClass,APBoxtype,ApColors,APAlignment,APSize,APLcol,APstyle,APgravity);
  { List of all object classes }
  ObjClasses=(FL_INVALID,FL_BUTTON, FL_LIGHTBUTTON,FL_ROUNDBUTTON, FL_ROUND3DBUTTON,
              FL_CHECKBUTTON, FL_BITMAPBUTTON, FL_PIXMAPBUTTON,FL_BITMAP, FL_PIXMAP,
              FL_BOX, FL_TEXT, FL_MENU, FL_CHART, FL_CHOICE, FL_COUNTER, FL_SLIDER, FL_VALSLIDER, FL_INPUT,
              FL_BROWSER,FL_DIAL,FL_TIMER,FL_CLOCK, FL_POSITIONER, FL_FREE,
              FL_XYPLOT,FL_FRAME, FL_LABELFRAME, FL_CANVAS, FL_GLCANVAS,
              FL_IMAGECANVAS, FL_FOLDER);
  { Properties in preamble }
  PreProps=(PPmagic,PPNrforms,PPUnitofMeasure,PPinvalid);
  { Properties of a form }
  FormProps=(FPName,FPWidth,FPHeight,FPnumObjs,FPinvalid);

Const
  { Names of all object properties }
  ContPropNames : Array[ContProps] of string[20] =
            ('class','type','box','boxtype','colors','alignment','style','size',
             'lcol','label','shortcut','resize','gravity','name','callback',
             'argument',
             'invalid');
  { Names of all object properties which must be checked.}
  AdjPropsNames : Array[AdjProps] of string[20] =
          ('class','boxtype','colors','alignment','size','lcol','style','gravity');
  { Names of all preamble properties }
  PrePropNames : Array[PreProps] of string[20] =
            ('Magic','Number of forms','Unit of measure','Invalid');
  { Names of all form properties }
  FormPropNames : Array[FormProps] of string[20] =
            ('Name','Width','Height','Number of Objects','Invalid');
  { Names of all object classes }
  FObjClassNames : Array[ObjClasses] of string[20]=
             ('FL_INVALID','BUTTON', 'LIGHTBUTTON','ROUNDBUTTON', 'ROUND3DBUTTON',
              'CHECKBUTTON', 'BITMAPBUTTON', 'PIXMAPBUTTON','BITMAP', 'PIXMAP',
              'BOX', 'TEXT', 'MENU', 'CHART', 'CHOICE', 'COUNTER', 'SLIDER', 'VALSLIDER', 'INPUT',
              'BROWSER','DIAL','TIMER','CLOCK', 'POSITIONER', 'FREE',
              'XYPLOT','FRAME', 'LABELFRAME', 'CANVAS', 'GLCANVAS',
              'IMAGECANVAS', 'FOLDER');

 { Default properties. If empty a property is ignored.
   To force setting of a property, put 'FL_FORCE' as a string.
   Mind : Case sensitive }

  DefProps : array[ObjClasses,AdjProps] of string[30] =
             (('FL_INVALID','','','','','','FL_NORMAL_STYLE','FL_FORCE'),
              ('BUTTON','FL_UP_BOX','FL_COL1 FL_COL1','FL_ALIGN_CENTER','','FL_LCOL','FL_NORMAL_STYLE','FL_FORCE'),
              ('LIGHTBUTTON','FL_UP_BOX','FL_COL1 FL_YELLOW','FL_ALIGN_CENTER','','FL_LCOL','FL_NORMAL_STYLE','FL_FORCE'),
              ('ROUNDBUTTON','FL_NO_BOX','FL_MCOL FL_YELLOW','FL_ALIGN_CENTER','','FL_LCOL','FL_NORMAL_STYLE','FL_FORCE'),
              ('ROUND3DBUTTON','FL_NO_BOX','FL_COL1 FL_BLACK','FL_ALIGN_CENTER','','FL_LCOL','FL_NORMAL_STYLE','FL_FORCE'),
              ('CHECKBUTTON','FL_NO_BOX','FL_COL1 FL_YELLOW','FL_ALIGN_CENTER','','FL_LCOL','FL_NORMAL_STYLE','FL_FORCE'),
              ('BITMAPBUTTON','FL_UP_BOX','FL_COL1 FL_BLUE','FL_ALIGN_BOTTOM','','FL_LCOL','FL_NORMAL_STYLE','FL_FORCE'),
              ('PIXMAPBUTTON','FL_UP_BOX','FL_COL1 FL_YELLOW','FL_ALIGN_BOTTOM','','FL_LCOL','FL_NORMAL_STYLE','FL_FORCE'),
              ('BITMAP','FL_NO_BOX','FL_COL1 FL_COL1','FL_ALIGN_BOTTOM','','FL_LCOL','FL_NORMAL_STYLE','FL_FORCE'),
              ('PIXMAP','FL_NO_BOX','FL_COL1 FL_COL1','FL_ALIGN_BOTTOM','','FL_LCOL','FL_NORMAL_STYLE','FL_FORCE'),
              ('BOX','','','','','','FL_NORMAL_STYLE','FL_FORCE'),
              ('TEXT','FL_FLAT_BOX','FL_COL1 FL_MCOL','FL_ALIGN_LEFT','','FL_LCOL','FL_NORMAL_STYLE','FL_FORCE'),
              ('MENU','FL_BORDER_BOX','FL_COL1 FL_MCOL','FL_ALIGN_CENTER','','FL_LCOL','FL_NORMAL_STYLE','FL_FORCE'),
              ('CHART','FL_BORDER_BOX','FL_COL1','FL_ALIGN_BOTTOM','','FL_LCOL','FL_NORMAL_STYLE','FL_FORCE'),
              ('CHOICE','FL_ROUNDED_BOX','FL_COL1 FL_LCOL','FL_ALIGN_LEFT','','FL_LCOL','FL_NORMAL_STYLE','FL_FORCE'),
              ('COUNTER','FL_UP_BOX','FL_COL1 FL_BLUE','FL_ALIGN_BOTTOM','','FL_LCOL','FL_NORMAL_STYLE','FL_FORCE'),
              ('SLIDER','FL_DOWN_BOX','FL_COL1 FL_COL1','FL_ALIGN_BOTTOM','','FL_LCOL','FL_NORMAL_STYLE','FL_FORCE'),
              ('VALSLIDER','FL_DOWN_BOX','FL_COL1 FL_COL1','FL_ALIGN_BOTTOM','','FL_LCOL','FL_NORMAL_STYLE','FL_FORCE'),
              ('INPUT','FL_DOWN_BOX','FL_COL1 FL_MCOL','FL_ALIGN_LEFT','','FL_LCOL','FL_NORMAL_STYLE','FL_FORCE'),
              ('BROWSER','FL_DOWN_BOX','FL_COL1 FL_YELLOW','FL_ALIGN_BOTTOM','FL_SMALL_FONT','FL_LCOL','FL_NORMAL_STYLE','FL_FORCE'),
              ('DIAL','FL_FLAT_BOX','FL_COL1 FL_RIGHT_BCOL','FL_ALIGN_BOTTOM','','FL_LCOL','FL_NORMAL_STYLE','FL_FORCE'),
              ('TIMER','FL_DOWN_BOX','FL_COL1 FL_RED','FL_ALIGN_CENTER','','FL_LCOL','FL_NORMAL_STYLE','FL_FORCE'),
              ('CLOCK','FL_UP_BOX','FL_INACTIVE_COL FL_BOTTOM_BCOL','FL_ALIGN_BOTTOM','','FL_BLACK','FL_NORMAL_STYLE','FL_FORCE'),
              ('POSITIONER','FL_DOWN_BOX','FL_COL1 FL_RED','FL_ALIGN_BOTTOM','','FL_LCOL','FL_NORMAL_STYLE','FL_FORCE'),
              ('FREE','','','','','','FL_NORMAL_STYLE','FL_FORCE'),
              ('XYPLOT','FL_FLAT_BOX','FL_COL1','FL_ALIGN_BOTTOM','','FL_LCOL','FL_NORMAL_STYLE','FL_FORCE'),
              ('FRAME','','FL_BLACK FL_COL1','','','FL_BLACK','FL_NORMAL_STYLE','FL_FORCE'),
              ('LABELFRAME','','','','','','FL_NORMAL_STYLE','FL_FORCE'),
              ('CANVAS','FL_NO_BOX','','FL_ALIGN_TOP','','','FL_NORMAL_STYLE','FL_FORCE'),
              ('GLCANVAS','','','','','','FL_NORMAL_STYLE','FL_FORCE'),
              ('IMAGECANVAS','','','','','','FL_NORMAL_STYLE','FL_FORCE'),
              ('FOLDER','','','','','','FL_NORMAL_STYLE','FL_FORCE'));

Type
  { object data type }
  PControl = ^TControl;
  TControl = Record
    Props : array[ContProps] of string;
    NextControl : PControl;
    end;

  { Form data type}
  PFormRec = ^TFormRec;
  TFormRec = Record
    Name : String;
    Width,Height : String[5];
    Controls : PControl;
    NextForm : PFormRec;
    end;
  { Callback data type }
  PCBrec = ^TCBrec;
  TCBrec = record
    name : string;
    next : PCBrec;
    end;
  { Property emitting procedures }
  EmitProp = Procedure (Data : PControl;ObjClass : ObjClasses);

Var
  OptionsSet : Array[1..NrOptions] Of Boolean;
  FileName : String;
  Infile,outfile : Text;
  LineNr : Longint;
  NrForms,NrControls : Longint;
  FormRoot : PFormRec;
  cbroot : pcbrec;
  { Default properties emitters }
  EmitProcs : array [AdjProps] of EmitProp;
  { Class specific property emitters. Nil pointers are ignored.}
  ClassEmitters : Array[ObjClasses] of EmitProp;

{ ------------------------------------------------------------------------
  Utilities Code
  ------------------------------------------------------------------------ }


Function IntTostr (s : Longint) : String;

var temp : String;

begin
  str(s,temp);
  IntToStr:=Temp;
end;


Procedure EmitError (Const s : String);

begin
  writeln (stderr,'Error: ',s);
  flush(stderr)
end;

Procedure EmitLineError (Const s : string);

begin
  EmitError('Line '+IntToStr(LineNr)+': '+s)
end;


{ ------------------------------------------------------------------------
  Option handling Code
  ------------------------------------------------------------------------ }


Procedure DoOptions;

Var i,j,k : byte;
    os : string;

Procedure ShowVersion;

begin
  Writeln ('fd2pascal : ',RevString);
  Halt(0);
end;

Procedure ShowUsage;

begin
  Writeln ('fd2pascal : usage :');
  writeln (' fd2pascal [options] filename');
  writeln (' Where [options] may be zero or more of :');
  writeln ('  -compensate    Emit size-compensation code.');
  writeln ('  -altformat     Emit code in alternate format.');
  writeln ('  -main          Emit program instead of unit.');
  writeln ('  -callback      Emit callback stubs.');
  writeln;
  halt(0);
end;

begin
  if paramcount=0 then
     ShowUsage;
  FileName:='';
  for i:=1 to paramcount do
    begin
    if paramstr(i)[1]<>'-' then
      If FileName<>'' then
        EmitError('Only one filename supported. Ignoring :'+paramstr(i))
      else
        Filename:=Paramstr(i)
    else
      begin
      os:=copy(paramstr(i),2,length(paramstr(i))-1);
      k:=NrOptions+1;
      for j:=0 to NrOptions do
        if os=options[j] then k:=j;
      if k=NrOptions+1 then
        EmitError('Option not recognised : '+paramstr(i))
      else
        if k=0 then ShowVersion else OptionsSet[k]:=True;
      end
    end; {for}
  if FileName='' then
    begin
    EmitError('No filename supplied. Exiting.');
    halt(1);
    end;
end;

{ ------------------------------------------------------------------------
  Code for reading the input file.
  ------------------------------------------------------------------------ }


Procedure OpenFile;
begin
  if pos('.fd',FileName)=0 then
    FileName:=FileName+'.fd';
  assign(infile,Filename);
{$i-}
  reset (infile);
{$i+}
  if ioresult<>0 then
    begin
    EmitError('Can''t open : '+filename);
    halt(1);
    end;
  LineNr:=0;
end;

Procedure CloseFile;

begin
  Close(infile);
end;

Procedure GetLine(Var S : String);

begin
  inc(LineNr);
  Readln(infile,s);
{$ifdef debug}
  writeln ('Reading line : ',linenr)
{$endif}
end;

Procedure ProcessPreAmbleLine (Const s: String);

var key,value : string;
    ppos : Longint;
    i,k : PreProps;
    code : Word;

begin
  if s='' then exit;
  ppos:=pos(':',s);
  if ppos=0 then
    exit;
  Key:=Copy(s,1,ppos-1);
  Value:=Copy(s,ppos+2,length(s)-ppos-1);
  k:=PPinvalid;
  for i:=PPmagic to PPinvalid do
    if key=PrePropNames[i] then k:=i;
  if k=PPinvalid then
    EmitLineError('Unknown keyword : '+Key)
  else
    Case K of
      PPMagic,
      PPunitofmeasure: ;
      PPnrforms: begin
               val(value,NrForms,code);
               if code<>0 then EmitLineError('Invalid number of forms');
               end;
    end;
end;

{ ------------------------------------------------------------------------
  Code for reading preamble.
  ------------------------------------------------------------------------ }


Procedure DoPreamble;

var line : String;

begin
{$ifdef debug}
  writeln ('Starting preamble');
{$endif}
  Getline (line);
  while pos('= FORM =',line)=0 do
    begin
    ProcessPreAmbleLine(line);
    GetLine(Line)
    end;
end;

{ ------------------------------------------------------------------------
  Code for reading 1 object.
  ------------------------------------------------------------------------ }


Procedure ProcessControlLine (PC : PControl; const S : String);

Var Key,Value : String;
    i,k : ContProps;
    ppos : word;

begin
  if s='' then exit;
  ppos:=pos(':',s);
  if ppos=0 then
    exit;
  Key:=Copy(s,1,ppos-1);
  Value:=Copy(s,ppos+2,length(s)-ppos-1);
  K:=CPInvalid;
  For i:=CPclass to CPInvalid do
    if ContPropNames[i]=Key then k:=i;
  if K=CPinvalid then
     begin
     EmitLineError('Unknown keyword'+key);
     exit
     end;
  PC^.props[k]:=value;
end;

Procedure ProcessControl (PC : PControl);

var line : String;

begin
{$ifdef debug}
  Writeln ('Starting Control');
{$endif}
  Getline(Line);
  while Line<>'' do
    begin
    ProcessControlLine (PC,line);
    Getline(Line);
    end;
  Getline(Line)
end;

{ ------------------------------------------------------------------------
  Code for reading 1 form.
  ------------------------------------------------------------------------ }

Procedure ProcessFormLine (PF : PFormRec; const S : String);

Var Key,Value : String;
    i,k : FormProps;
    ppos,code : word;

begin
  if s='' then exit;
  ppos:=pos(':',s);
  if ppos=0 then
    exit;
  Key:=Copy(s,1,ppos-1);
  Value:=Copy(s,ppos+2,length(s)-ppos-1);
  K:=FPInvalid;
  For i:=FPName to FPInvalid do
    if FormPropNames[i]=Key then k:=i;
  if K=FPinvalid then
     begin
     EmitLineError('Unknown keyword'+key);
     exit
     end;
  case k of
    FPname    : PF^.name:=value;
    FPWidth   : PF^.width:=value;
    FPHeight  : PF^.height:=value;
    FPNumObjs : begin
                val(value,Nrcontrols,code);
                If Code<>0 then EmitLineError('Invalid number of objects : '+value)
                end;
    end;
end;

Procedure ProcessForm (PF : PFormRec);

Var line : String;
    CurrentControl : PControl;
    I : Integer;

begin
{$ifdef debug}
  writeln('Starting form');
{$endif}
  NrControls:=0;
  with PF^ do
    begin
    name:='';
    Width:='';
    Height:='';
    Controls:=nil;
    GetLine(Line);
    while line<>'' do
      begin
      ProcessFormLine(PF,Line);
      GetLine(Line);
      end;
    Getline(Line);
    If NrControls=0 then
      Controls:=nil
    else
      begin
      New (Controls);
      CurrentControl:=Controls;
      for i:=1 to nrcontrols do
        begin
        ProcessControl(CurrentControl);
        if i<NrControls then
          New(CurrentControl^.NextControl)
        else
          CurrentControl^.NextControl:=nil;
        CurrentControl:=CurrentControl^.NextControl
        end; { for }
      end; { Else }
    end; { With }
end;

{ ------------------------------------------------------------------------
  Code for reading the forms.
  ------------------------------------------------------------------------ }


Procedure DoForms;

Var
    i : Longint;
    CurrentForm: PformRec;

begin
  FormRoot:=Nil;
  if NrForms=0 then exit;
  new(FormRoot);
  Currentform:=FormRoot;
  for i:=1 to nrforms do
     begin
     ProcessForm (CurrentForm);
     If i=nrforms then
       Currentform^.NextForm:=nil
     else
       New(CurrentForm^.NextForm);
     CurrentForm:=CurrentForm^.NextForm;
     end;
end;

{ ------------------------------------------------------------------------
  Code for reading the postamble.
  ------------------------------------------------------------------------ }


Procedure DoPostamble;

begin
end;

{ ------------------------------------------------------------------------
  Code for writing the output file.
  ------------------------------------------------------------------------ }

Procedure OpenOutFile;

var info : stat;

begin
  FileName:=Copy(Filename,1,Length(Filename)-3)+'.pp';
  if fpstat(FileName,info)<>-1 Then
    begin
    { File exists, move to .bak}
      fplink (FileName,FileName+'.bak');
      fpunlink(FileName);
    end;

  assign(outfile,filename);
{$i-}
  rewrite(outfile);
{$i+}
  if ioresult<>0 then
    begin
    EmitError('Couldn''t open output file : '+filename);
    halt(1)
    end;
end;

Procedure CloseOutFile;

begin
 Close(OutFile);
end;

{ ------------------------------------------------------------------------
  Code to emit Header/variable/type declarations
  ------------------------------------------------------------------------ }


Procedure EmitType (fp : Pformrec);

var cp : PControl;

begin
  writeln (OutFile,'  TFD_',fp^.name,' = record');
  writeln (OutFile,'    ',fp^.name,' : PFL_FORM;');
  writeln (OutFile,'    vdata : Pointer;');
  writeln (OutFile,'    ldata : Longint;');
  cp:=fp^.controls;
  {Skip first control, is formbackground }
  if cp<>nil then cp:=cp^.nextcontrol;
  while cp<>nil do
    begin
    if cp^.props[CPclass]<>'FL_END_GROUP' then
      begin
      write (Outfile,'    ',cp^.props[CPname]);
      if cp^.nextcontrol<>nil then
        writeln (OutFile,',')
      else
        writeln (OutFile,' : PFL_OBJECT;');
      end;
    cp:=cp^.nextcontrol;
    end;
  writeln (OutFile,'    end;');
  writeln (OutFile,'  PFD_',fp^.name,' = ^TFD_',fp^.name,';');
  writeln (OutFile);
end;

Procedure EmitVar (fp : Pformrec);

var cp : PControl;

begin
  writeln (OutFile,'  ',fp^.name,' : PFL_FORM;');
  cp:=fp^.controls;
  {Skip first control, is formbackground }
  if cp<>nil then cp:=cp^.nextcontrol;
  while cp<>nil do
    begin
    if cp^.props[CPclass]<>'FL_END_GROUP' then
      begin
      write (Outfile,'  ',cp^.props[CPname]);
      if cp^.nextcontrol<>nil then
        writeln (OutFile,',')
      else
        writeln (OutFile,' : PFL_OBJECT;');
      end;
    cp:=cp^.nextcontrol;
    end;
  writeln (OutFile);
end;

Procedure EmitHeader;

var fp : PFormRec;

begin
  if OptionsSet[2] then
    write   (OutFile,'Program ')
  else
    write   (OutFile,'Unit ');
  writeln (OutFile,basename(filename,'.pp'),';');
  writeln (OutFile);
  writeln (OutFile,'{ Form definition file generated by fd2pascal }');
  writeln (Outfile);
  if not OptionsSet[2] then
     begin
     writeln (OutFile,'Interface');
     writeln (OutFile);
     end;
  writeln (OutFile,'Uses forms;');
  writeln (OutFile);
  writeln (OutFile,'  { Variable / Type definitions. }');
  if Optionsset[3] then
    writeln (OutFile,'Var')
  else
    writeln (OutFile,'Type');
  fp:=FormRoot;
  While fp<>nil do
    begin
    if not optionsset[3] then
      EmitType(fp) { Emit Type definitions }
    else
      EmitVar(fp); { Emit Variable declaration}
    fp:=fp^.nextform;
    end;
  if not optionsset[2] then
    begin
    { No program, we must emit interface stuff }
    if not (optionsset[3]) then
      begin
      { Emit normal interface declarations
        -> functions }
      fp:=formroot;
      while fp<>nil do
        begin
        with fp^ do
          writeln (OutFile,'Function create_form_',name,' : PFD_',name,';');
        fp:=fp^.nextform;
        end;
      end
    else
      begin
      { Emit alternate interface declaration
        -> 1 function to create all forms.}
      writeln (OutFile,'Procedure Create_The_Forms;');
      end;
    writeln (OutFile);
    writeln (OutFile,'Implementation');
    end
  else
    begin
    { We must make a program. }
    if not(optionsset[3]) then
      begin
      { Normal format, so we need to emit variables for the forms.}
      writeln (OutFile,'Var');
      fp:=formroot;
      while fp<>nil do
        begin
        writeln (OutFile,'  ',fp^.name,' : PFD_',fp^.name,';');
        fp:=fp^.nextform;
        end;
      writeln (OutFile);
      end;
    end;
  writeln (OutFile);
end;

{ ------------------------------------------------------------------------
  Code to emit footer/main program
  ------------------------------------------------------------------------ }


Procedure EmitCreateforms;

var fp : PFormRec;

begin
  writeln (OutFile,'Procedure Create_The_Forms;');
  writeln (OutFile);
  writeln (OutFile,'begin');
  fp:=FormRoot;
  while fp<>nil do
    begin
    writeln(OutFile,'create_form_',fp^.name,';');
    fp:=fp^.nextform;
    end;
  writeln (outFile,'End;');
  writeln (OutFile);
end;

Procedure EmitAlternateMain;
begin
  { Alternate format, we just call creatallforms to create all forms}
  writeln (OutFile,'Create_The_Forms;');
  writeln (OutFile,'  fl_show_form(',formroot^.name,
                   ',FL_PLACE_CENTER,FL_FULLBORDER,''',
                   FormRoot^.name,''');');
end;

Procedure EmitMain;

var fp : PFormRec;

begin
  { variables are emitted in the header }
  fp:=formroot;
  { Create all forms }
  while fp<>nil do
    begin
    writeln (OutFile,'  ',fp^.name,' :=Create_Form_',fp^.name,';');
    fp:=fp^.nextform;
    end;
  { Show the first form }
  writeln (OutFile,'  fl_show_form(',formroot^.name,'^.',Formroot^.name,
                   ',FL_PLACE_CENTER,FL_FULLBORDER,''',
                   FormRoot^.name,''');');
end;

Procedure EmitFooter;
begin
  if OptionsSet[3] then {Alternate format.}
     EmitCreateForms;
  if Optionsset[2] then
    begin
    {Emit Main Program}
    writeln (OutFile);
    writeln (OutFile,'Begin');
    writeln (OutFile,'  fl_initialize (@argc,argv,''',
                     basename(Filename,'.pp'),''',nil,0);');
    if Not(OptionsSet[3]) then
      EmitMain
    else
      EmitAlternateMain;
    writeln (OutFile,'  fl_do_forms;');
    end
  else
    writeln (OutFile,'begin');
  writeln (OutFile,'end.')
end;




{ ------------------------------------------------------------------------
  Code to emit properties
  ------------------------------------------------------------------------ }


Function EmitString(S : string) : String;

var temp : String;
    i : longint;

begin
  temp:='''';
  for i:=1 to length(s) do
    if s[i]<>'''' then temp:=temp+s[i] else temp:=temp+'''''';
  Temp:=temp+'''';
  EmitString:=temp;
end;

Procedure EmitBoxtype (cp : PControl;ObjClass : ObjClasses);

begin
{$ifdef debug}
  writeln ('EmitBoxType called with args:');
  writeln (cp^.props[cpboxtype]);
  writeln (defprops[objclass,APboxtype]);
  writeln ('for object : ',defprops[objclass,apclass]);
  writeln ('With object : ',cp^.props[cpclass]);
{$endif}
  if cp^.props[cpboxtype]<>defprops[objclass,APboxtype] then
    writeln (OutFile,'    fl_set_object_boxtype(obj,',
                    cp^.props[cpboxtype],');')
end;

Procedure EmitColors (cp : PControl;ObjClass : ObjClasses);

var temp : string;

begin
  if cp^.props[cpcolors]<>defprops[objclass,APcolors] then
    begin
    temp:=cp^.props[cpcolors];
    if pos(' ',temp)=0 then exit;
    temp[pos(' ',temp)]:=',';
    writeln (OutFile,'    fl_set_object_color(obj,',temp,');');
    end;
end;

Procedure EmitAlignment (cp : PControl;ObjClass : ObjClasses);

begin
 if cp^.props[cpalignment]<>defprops[objclass,APalignment] then
    writeln (OutFile,'    fl_set_object_alignment(obj,',
                     cp^.props[cpalignment],');');
end;

Procedure EmitLcol (cp : PControl;ObjClass : ObjClasses);

begin
 if cp^.props[cplcol]<>defprops[objclass,APlcol] then
    writeln (OutFile,'    fl_set_object_lcol(obj,',
                     cp^.props[cplcol],');');
end;


Procedure EmitSize (cp : PControl;ObjClass : ObjClasses);

begin
 if cp^.props[cpsize]<>defprops[objclass,APsize] then
    writeln (OutFile,'    fl_set_object_lsize(obj,',
                     cp^.props[cpsize],');');
end;

Procedure EmitStyle (cp : PControl;ObjClass : ObjClasses);

begin
 if cp^.props[cpstyle]<>defprops[objclass,APstyle] then
    writeln (OutFile,'    fl_set_object_lstyle(obj,',
                     cp^.props[cpstyle],');');
end;

Procedure EmitGravity (cp : PControl;ObjClass : ObjClasses);

var temp: string;

begin
 if cp^.props[cpstyle]<>'FL_NoGravity FL_NoGravity' then
    begin
    temp:=cp^.props[cpstyle];
    if pos(' ',temp)=0 then exit;
    temp[pos(' ',temp)]:=',';
    writeln (OutFile,'    fl_set_object_gravity(obj,',
                     temp,');');
    end;
end;


Procedure EmitProperties (Cp : PControl; Objclass : ObjClasses);

Var i : AdjProps;

begin
  for i:=APboxtype to APgravity do
    if DefProps[ObjClass,i]<>'' then
       EmitProcs[i](cp,objclass);
end;

{ ------------------------------------------------------------------------
  Code to emit objects
  ------------------------------------------------------------------------ }

Procedure EmitObject(cp : PControl);

var temp : string;
    I : Longint;
    j,k : ObjClasses;

begin
with cp^ do
  begin
  temp:=lowercase(props[CPclass]);
  delete(temp,1,3);
  if temp='begin_group' then
    begin
    writeln (OutFile);
    write (OutFile,'  ');
    if not (Optionsset[3]) then Write (OutFile,'fdui^.');
    writeln (OutFile,props[cpname],':=fl_bgn_group;');
    exit;
    end
  else if temp='end_group' then
    begin
    writeln (OutFile,'  fl_end_group;');
    writeln (OutFile);
    exit;
    end;
  { Normal object. Emit creation code. }
  write (OutFile,'  obj:=fl_add_',temp,' (FL_',props[Cptype],',');
  temp:=props[cpbox];
  for i:=1 to 3 do
    begin
    write (OutFile,copy(temp,1,pos(' ',temp)-1),',');
    delete (temp,1,pos(' ',temp));
  end;
  writeln (OutFile,temp,',',EmitString(props[cplabel]),');');
  { Emit Callback code if needed }
  if props[cpcallback]<>'' then
    begin
    write (OutFile,'    fl_set_object_callback(obj,PFL_CALLBACKPTR(@');
    write (OutFile,props[CPcallback],'),');
    if props[CPargument]<>'' then
       writeln (OutFile,props[CPargument],');')
    else
       writeln (OutFile,'0);');
    end;
  { If known object, start emitting properties }
  temp:=props[CPclass];
  delete(temp,1,3);
  k:=FL_INVALID;
  for j:=FL_BUTTON to FL_FOLDER do
    if temp=DefProps[j,apclass] then k:=j;
  if k<>FL_INVALID then
     begin
     { Emit defaults }
     EmitProperties (cp,k);
     { If A class-specific emitter exists, call it.}
     if Assigned(ClassEmitters[k]) then
       ClassEmitters[k] (cp,k);
     end;
  { Assign to needed object. }
  if Optionsset[3] then
    Writeln (OutFile,'  ',props[cpname],':=obj;')
  else
    Writeln (OutFile,'  fdui^.',props[cpname],':=obj;');
  end;
end;

{ ------------------------------------------------------------------------
  Code to emit forms
  ------------------------------------------------------------------------ }

Procedure EmitForm(fp : PFormRec);

Var
cp : PControl;

begin
with fp^ do
  begin
  if Optionsset[3] then
    begin
    writeln (OutFile,'Procedure create_form_',name,';');
    writeln (OutFile);
    writeln (OutFile,'Var obj : PFL_OBJECT;');
    writeln (OutFile);
    writeln (OutFile,'Begin');
    writeln (OutFile,'  If ',name,'<>nil then exit;');
    write   (OutFile,'  ',name);
    end
  else
    begin
    writeln (OutFile,'Function create_form_',name,' : PFD_',name,';');
    writeln (OutFile);
    writeln (OutFile,'Var obj : PFL_OBJECT;');
    writeln (OutFile,'    fdui : PFD_',name,';');
    writeln (OutFile);
    writeln (OutFile,'Begin');
    writeln (OutFile,'  New(fdui);');
    write (OutFile,'  fdui^.',name);
    end;
  writeln (OutFile,':=fl_bgn_form(FL_NO_BOX,'
                                                  ,width,','
                                                  ,height,');');
  cp:=controls;
  writeln (OutFile,'  obj:=fl_add_box(',cp^.props[CPboxtype],',0,0,',
                                      width,',',
                                      height,',',
                                      EmitString (cp^.props[CPname]),');');
  cp:=cp^.nextcontrol;
  { Emit all objects }
  while cp<>nil do
    begin
    EmitObject(cp);
    cp:=cp^.nextcontrol;
    end;
  writeln (OutFile,'  fl_end_form;');
  if Optionsset[4] then
    begin
    { Emit Compensation code }
    write (OutFile,'  fl_adjust_form_size(');
    if not(OptionsSet[3]) then write (OutFile,'fdui^.');
    writeln(OutFile,fp^.name,');');
    end;
  if not(OptionsSet[3]) then
    begin
    writeln (OutFile,'  fdui^.',fp^.name,'^.fdui:=fdui;');
    writeln (OutFile,'  create_form_',fp^.name,':=fdui;');
    end;
  writeln (OutFile,'end;');
  writeln (OutFile);
  end;
end;

Procedure EmitForms;

var
  fp : PformRec;
begin
  { Start emitting forms }
  fp:=Formroot;
  while fp<>nil do
    begin
    EmitForm(fp);
    fp:=fp^.nextform;
    end;
end;

{ ------------------------------------------------------------------------
  Code to emit callbacks
  ------------------------------------------------------------------------ }

Procedure CollectCallbacks;

Var CurrentCb,CBwalk : PCBrec;
    fp : PformRec;
    cp : PControl;

begin
  CbRoot:=nil;
  CurrentCB:=cbroot;
  fp:=formroot;
  while fp<>nil do
    begin
    cp:=fp^.controls;
    while cp<>nil do
      begin
      if cp^.props[CPcallback]<>'' then
        if cbroot<>nil then
          begin
          cbwalk:=cbroot;
          while cbwalk<>nil do
            if upcase(cbwalk^.name)=upcase(cp^.props[CPcallback]) then
              break
            else
              cbwalk:=cbwalk^.next;
          if cbwalk=nil then
            begin
            new(currentcb^.next);
            currentcb:=currentcb^.next;
            currentcb^.name:=cp^.props[CPcallback];
            currentcb^.next:=nil;
            end;
          end
        else
          begin
          new(cbroot);
          currentcb:=cbroot;
          cbroot^.name:=cp^.props[CPcallback];
          cbroot^.next:=nil;
          end;
      cp:=cp^.nextcontrol;
      end;
    fp:=fp^.nextform;
    end;
end;

Procedure EmitCallback (Const s : string);

begin
  writeln (OutFile,'Procedure ',s,' (Sender: PFL_OBJECT; Data : Longint); export;');
  writeln (OutFile);
  writeln (OutFile,'begin');
  writeln (OutFile,'  { Place your code here }');
  writeln (OutFile,'end;');
  writeln (OutFile);
end;

Procedure EmitCallBacks;

var cb : pcbrec;

begin
  { See if we must emit callback stubs }
  If Optionsset[1] then
    begin
    cb:=cbroot;
    while cb<>nil do
      begin
      EmitCallBack(cb^.Name);
      cb:=cb^.next;
      end;
    end;
end;


{ ------------------------------------------------------------------------
  EmitterTable initialization Code
  ------------------------------------------------------------------------ }

Procedure EmitDummy (cp : PControl;ObjClass : ObjClasses);

begin
end;

Procedure InitEmitters;

var i : objclasses;

begin
  EmitProcs[APClass]:=@EmitDummy;
  EmitProcs[APBoxtype]:=@EmitBoxType;
  EmitProcs[APColors]:=@EmitColors;
  EmitProcs[APAlignment]:=@EmitAlignment;
  EmitProcs[APlcol]:=@EmitLcol;
  EmitProcs[APsize]:=@EmitSize;
  EmitProcs[APStyle]:=@EmitStyle;
  EmitProcs[APgravity]:=@EmitGravity;
  for i:=FL_INVALID to FL_FOLDER do
    ClassEmitters[i]:=EmitProp(Nil);
end;

{ ------------------------------------------------------------------------
  Main program Code
  ------------------------------------------------------------------------ }


begin
  { Process options }
  DoOptions;
  { Read input file }
  OpenFile;
  DoPreamble;
  DoForms;
  DoPostamble;
  CloseFile;
  { Write output file }
  OpenOutfile;
  InitEmitters;
  CollectCallbacks;
  EmitHeader;
  EmitCallbacks;
  EmitForms;
  EmitFooter;
  CloseOutFile;
end.
