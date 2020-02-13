{$mode objfpc}{$H+}
{$define allpackages}
program fpmake;

{$IFDEF OS2}
 {$DEFINE NO_UNIT_PROCESS}
{$ENDIF OS2}

{$IFDEF GO32V2}
 {$DEFINE NO_UNIT_PROCESS}
{$ENDIF GO32V2}

{$ifndef NO_UNIT_PROCESS}
  {$define HAS_UNIT_PROCESS}
{$endif NO_UNIT_PROCESS}

uses
  fpmkunit,
  classes,
{$IFDEF HAS_UNIT_PROCESS}
  process,
{$ENDIF HAS_UNIT_PROCESS}
  sysutils;


Var
  TBuild,T : TTarget;
  PBuild,P : TPackage;
  D : TDependency;
  I : Integer;

(*

The include files are generated with the following commands:

rm fpmake_proc.inc fpmake_add.inc ; /bin/ls -1 */fpmake.pp| while read file; do cleanedname=`dirname $file | sed -e 's+-+_+g'` ; if ! `grep -i "^procedure add_$cleanedname" $file >/dev/null` ; then printf 'procedure add_%s(const ADirectory: string);\nbegin\n  with Installer do\n{$include %s}\nend;\n\n' $cleanedname $file >> fpmake_proc.inc; else printf '{$include %s}\n\n' $file >> fpmake_proc.inc; fi; echo "  add_$cleanedname(ADirectory+IncludeTrailingPathDelimiter('$cleanedname'));" >> fpmake_add.inc; done

*)

{$include fpmake_proc1.inc}

procedure add_fpc(const ADirectory: string);

begin
{$include fpmake_add1.inc}

end;

begin
  add_packages_comandlineoptions();
  add_fpc('');
  Installer.Run;
end.




