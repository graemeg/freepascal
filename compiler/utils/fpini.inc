{$ifdef Delphi}
const fpini : array[0..4] of string[240]=(
{$else Delphi}
const fpini : array[0..4,1..240] of char=(
{$endif Delphi}
  '[Compile]'#013#010+
  'CompileMode=DEBUG'#013#010+
  #013#010+
  '[Editor]'#013#010+
  'DefaultTabSize=8'#013#010+
  'DefaultFlags=20599'#013#010+
  'DefaultSaveExt=.pas'#013#010+
  'DefaultIndentSize=1'#013#010+
  #013#010+
  '[Highlight]'#013#010+
  'Exts="*.pas;*.pp;*.inc"'#013#010+
  'NeedsTabs="make*;make*.*"'#013#010+
  #013#010+
  '[SourcePath]'#013#010+
  'SourceList=""'#013#010+
  #013#010+
  '[Mouse]'#013#010+
  'DoubleDel','ay=8'#013#010+
  'ReverseButtons=0'#013#010+
  'AltClickAction=6'#013#010+
  'CtrlClickAction=1'#013#010+
  #013#010+
  '[Search]'#013#010+
  'FindFlags=4'#013#010+
  #013#010+
  '[Breakpoints]'#013#010+
  'Count=0'#013#010+
  #013#010+
  '[Watches]'#013#010+
  'Count=0'#013#010+
  #013#010+
  '[Preferences]'#013#010+
  'DesktopFileFlags=209'#013#010+
  'CenterCurrentLineWhileDebugging=1'#013#010+
  'AutoSaveFlags=7'#013#010+
  'MiscOptions=6'#013#010,
  'DesktopLocation=1'#013#010+
  #013#010+
  '[Misc]'#013#010+
  'ShowReadme=1'#013#010+
  #013#010+
  '[Files]'#013#010+
  'OpenExts="*.pas;*.pp;*.inc"'#013#010+
  #013#010+
  '[Tools]'#013#010+
  'Title1="svn ~u~p (curr. dir)"'#013#010+
  'Program1="svn"'#013#010+
  'Params1="up $CAP_MSG()"'#013#010+
  'HotKey1=23296'#013#010+
  'Title2="svn c~i~ (curr. dir)"'#013#010+
  'Program2="svn"'#013#010+
  'Params2="ci ','$CAP_MSG()"'#013#010+
  'HotKey2=23552'#013#010+
  'Title3="svn ~d~iff"'#013#010+
  'Program3="svn"'#013#010+
  'Params3="diff $CAP_MSG() $EDNAME"'#013#010+
  'HotKey3=23808'#013#010+
  'Title4="svn ~l~og"'#013#010+
  'Program4="svn"'#013#010+
  'Params4="log $CAP_MSG() $EDNAME"'#013#010+
  'HotKey4=34560'#013#010+
  'Title5="svn ~b~lame"'#013#010+
  'Program5="svn"'#013#010+
  'Pa','rams5="blame $CAP_MSG() $EDNAME"'#013#010+
  'HotKey5=34816'#013#010+
  'Title6="svn ~a~dd"'#013#010+
  'Program6="svn"'#013#010+
  'Params6="add $CAP_MSG() $EDNAME"'#013#010+
  'HotKey6=0'#013#010
);
