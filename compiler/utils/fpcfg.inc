{$ifdef Delphi}
const fpcfg : array[0..2] of string[240]=(
{$else Delphi}
const fpcfg : array[0..2,1..240] of char=(
{$endif Delphi}
  '#IFDEF NORMAL'#010+
  ' -T%fpctargetos%'#010+
  ' -Sg'#010+
  ' -O1'#010+
  ' -Oppentium3'#010+
  ' -Ratt'#010+
  ' -Fu%basepath%\units\%fpctarget%-%fpctargetos%'#010+
  ' -Fu%basepath%\units\%fpctarget%-%fpctargetos%\*'#010+
  ' -Fu%basepath%\units\%fpctarget%-%fpctargetos%\rtl'#010+
  ' -g-'#010+
  ' -p-'#010+
  ' -b-'#010+
  '#ENDIF'#010+
  #010+
  '#IFDEF DE','BUG'#010+
  ' -T%fpctargetos%'#010+
  ' -Sg'#010+
  ' -Cr'#010+
  ' -Ci'#010+
  ' -Co'#010+
  ' -Oppentium3'#010+
  ' -Ratt'#010+
  ' -Fu%basepath%\units\%fpctarget%-%fpctargetos%'#010+
  ' -Fu%basepath%\units\%fpctarget%-%fpctargetos%\*'#010+
  ' -Fu%basepath%\units\%fpctarget%-%fpctargetos%\rtl'#010+
  ' -g'#010+
  ' -p-'#010+
  ' -b-'#010+
  '#ENDIF'#010+
  #010+
  '#IFDEF REL','EASE'#010+
  ' -T%fpctargetos%'#010+
  ' -Sg'#010+
  ' -O2'#010+
  ' -Oppentium3'#010+
  ' -Ratt'#010+
  ' -Fu%basepath%\units\%fpctarget%-%fpctargetos%'#010+
  ' -Fu%basepath%\units\%fpctarget%-%fpctargetos%\*'#010+
  ' -Fu%basepath%\units\%fpctarget%-%fpctargetos%\rtl'#010+
  ' -g-'#010+
  ' -p-'#010+
  ' -b-'#010+
  '#ENDIF'#010
);
