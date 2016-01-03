{$ifndef ALLPACKAGES}
{$mode objfpc}{$H+}
program fpmake;

uses fpmkunit;

Var
  T,TBuild : TTarget;
  P : TPackage;
  i : Integer;
begin
  With Installer do
    begin
{$endif ALLPACKAGES}

    P:=AddPackage('winunits-jedi');
{$ifdef ALLPACKAGES}
<<<<<<< HEAD
<<<<<<< HEAD
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}
    P.Version:='3.1.1';
    P.OSes:=[win32,win64];
    P.Options.Add('-Ur');
    P.Author := 'Marcel van Brakel, Jedi-apilib team';
    P.License := 'LGPL with modification/MPL dual licensed ';
    P.HomepageURL := 'www.freepascal.org';
    P.Email := '';
    P.Description := 'Very complete set of Windows units by Jedi Apilib';
    P.NeedLibC:= true;
=======
=======
>>>>>>> origin/fixes_2_2
    P.Directory:='winunits-jedi';
{$endif ALLPACKAGES}
    P.Version:='2.2.4';
    P.OSes:=[win32,win64];
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

    P.Dependencies.Add('winunits-base');

    P.SourcePath.Add('src');

    T:=P.Targets.AddImplicitUnit('jwaaccctrl.pas');
    T:=P.Targets.AddImplicitUnit('jwaaclapi.pas');
    T:=P.Targets.AddImplicitUnit('jwaaclui.pas');
    T:=P.Targets.AddImplicitUnit('jwaactiveds.pas');
    T:=P.Targets.AddImplicitUnit('jwaactivex.pas');
    T:=P.Targets.AddImplicitUnit('jwaadsdb.pas');
    T:=P.Targets.AddImplicitUnit('jwaadserr.pas');
    T:=P.Targets.AddImplicitUnit('jwaadshlp.pas');
    T:=P.Targets.AddImplicitUnit('jwaadsnms.pas');
    T:=P.Targets.AddImplicitUnit('jwaadsprop.pas');
    T:=P.Targets.AddImplicitUnit('jwaadssts.pas');
    T:=P.Targets.AddImplicitUnit('jwaadstlb.pas');
    T:=P.Targets.AddImplicitUnit('jwaadtgen.pas');
    T:=P.Targets.AddImplicitUnit('jwaaf_irda.pas');
    T:=P.Targets.AddImplicitUnit('jwaatalkwsh.pas');
    T:=P.Targets.AddImplicitUnit('jwaauthif.pas');
    T:=P.Targets.AddImplicitUnit('jwaauthz.pas');
    T:=P.Targets.AddImplicitUnit('jwabatclass.pas');
    T:=P.Targets.AddImplicitUnit('jwabits1_5.pas');
    T:=P.Targets.AddImplicitUnit('jwabitscfg.pas');
    T:=P.Targets.AddImplicitUnit('jwabitsmsg.pas');
    T:=P.Targets.AddImplicitUnit('jwabits.pas');
    T:=P.Targets.AddImplicitUnit('jwablberr.pas');
    T:=P.Targets.AddImplicitUnit('jwabluetoothapis.pas');
    T:=P.Targets.AddImplicitUnit('jwabthdef.pas');
    T:=P.Targets.AddImplicitUnit('jwabthsdpdef.pas');
    T:=P.Targets.AddImplicitUnit('jwabugcodes.pas');
    T:=P.Targets.AddImplicitUnit('jwacarderr.pas');
    T:=P.Targets.AddImplicitUnit('jwacderr.pas');
    T:=P.Targets.AddImplicitUnit('jwacmnquery.pas');
    T:=P.Targets.AddImplicitUnit('jwacolordlg.pas');
    T:=P.Targets.AddImplicitUnit('jwacplext.pas');
    T:=P.Targets.AddImplicitUnit('jwacpl.pas');
    T:=P.Targets.AddImplicitUnit('jwacryptuiapi.pas');
    T:=P.Targets.AddImplicitUnit('jwadbt.pas');
    T:=P.Targets.AddImplicitUnit('jwadde.pas');
    T:=P.Targets.AddImplicitUnit('jwadhcpcsdk.pas');
    T:=P.Targets.AddImplicitUnit('jwadhcpsapi.pas');
    T:=P.Targets.AddImplicitUnit('jwadhcpssdk.pas');
    T:=P.Targets.AddImplicitUnit('jwadlgs.pas');
    T:=P.Targets.AddImplicitUnit('jwadsadmin.pas');
    T:=P.Targets.AddImplicitUnit('jwadsclient.pas');
    T:=P.Targets.AddImplicitUnit('jwadsgetdc.pas');
    T:=P.Targets.AddImplicitUnit('jwadskquota.pas');
    T:=P.Targets.AddImplicitUnit('jwadsquery.pas');
    T:=P.Targets.AddImplicitUnit('jwadsrole.pas');
    T:=P.Targets.AddImplicitUnit('jwadssec.pas');
    T:=P.Targets.AddImplicitUnit('jwaerrorrep.pas');
    T:=P.Targets.AddImplicitUnit('jwaexcpt.pas');
    T:=P.Targets.AddImplicitUnit('jwafaxdev.pas');
    T:=P.Targets.AddImplicitUnit('jwafaxext.pas');
    T:=P.Targets.AddImplicitUnit('jwafaxmmc.pas');
    T:=P.Targets.AddImplicitUnit('jwafaxroute.pas');
    T:=P.Targets.AddImplicitUnit('jwagpedit.pas');
    T:=P.Targets.AddImplicitUnit('jwahherror.pas');
    T:=P.Targets.AddImplicitUnit('jwahtmlguid.pas');
    T:=P.Targets.AddImplicitUnit('jwahtmlhelp.pas');
    T:=P.Targets.AddImplicitUnit('jwaiaccess.pas');
    T:=P.Targets.AddImplicitUnit('jwaiadmext.pas');
    T:=P.Targets.AddImplicitUnit('jwaicmpapi.pas');
    T:=P.Targets.AddImplicitUnit('jwaiiscnfg.pas');
    T:=P.Targets.AddImplicitUnit('jwaimagehlp.pas');
    T:=P.Targets.AddImplicitUnit('jwaimapierror.pas');
    T:=P.Targets.AddImplicitUnit('jwaimapi.pas');
    T:=P.Targets.AddImplicitUnit('jwaime.pas');
    T:=P.Targets.AddImplicitUnit('jwaioevent.pas');
    T:=P.Targets.AddImplicitUnit('jwaipexport.pas');
    T:=P.Targets.AddImplicitUnit('jwaiphlpapi.pas');
    T:=P.Targets.AddImplicitUnit('jwaipifcons.pas');
    T:=P.Targets.AddImplicitUnit('jwaipinfoid.pas');
    T:=P.Targets.AddImplicitUnit('jwaiprtrmib.pas');
    T:=P.Targets.AddImplicitUnit('jwaiptypes.pas');
    T:=P.Targets.AddImplicitUnit('jwaisguids.pas');
    T:=P.Targets.AddImplicitUnit('jwaissper16.pas');
    T:=P.Targets.AddImplicitUnit('jwalmaccess.pas');
    T:=P.Targets.AddImplicitUnit('jwalmalert.pas');
    T:=P.Targets.AddImplicitUnit('jwalmapibuf.pas');
    T:=P.Targets.AddImplicitUnit('jwalmat.pas');
    T:=P.Targets.AddImplicitUnit('jwalmaudit.pas');
    T:=P.Targets.AddImplicitUnit('jwalmconfig.pas');
    T:=P.Targets.AddImplicitUnit('jwalmcons.pas');
    T:=P.Targets.AddImplicitUnit('jwalmdfs.pas');
    T:=P.Targets.AddImplicitUnit('jwalmerrlog.pas');
    T:=P.Targets.AddImplicitUnit('jwalmerr.pas');
    T:=P.Targets.AddImplicitUnit('jwalmjoin.pas');
    T:=P.Targets.AddImplicitUnit('jwalmmsg.pas');
    T:=P.Targets.AddImplicitUnit('jwalmremutl.pas');
    T:=P.Targets.AddImplicitUnit('jwalmrepl.pas');
    T:=P.Targets.AddImplicitUnit('jwalmserver.pas');
    T:=P.Targets.AddImplicitUnit('jwalmshare.pas');
    T:=P.Targets.AddImplicitUnit('jwalmsname.pas');
    T:=P.Targets.AddImplicitUnit('jwalmstats.pas');
    T:=P.Targets.AddImplicitUnit('jwalmsvc.pas');
    T:=P.Targets.AddImplicitUnit('jwalmuseflg.pas');
    T:=P.Targets.AddImplicitUnit('jwalmuse.pas');
    T:=P.Targets.AddImplicitUnit('jwalmwksta.pas');
    T:=P.Targets.AddImplicitUnit('jwaloadperf.pas');
    T:=P.Targets.AddImplicitUnit('jwalpmapi.pas');
    T:=P.Targets.AddImplicitUnit('jwamciavi.pas');
    T:=P.Targets.AddImplicitUnit('jwamprerror.pas');
    T:=P.Targets.AddImplicitUnit('jwamsidefs.pas');
    T:=P.Targets.AddImplicitUnit('jwamsi.pas');
    T:=P.Targets.AddImplicitUnit('jwamsiquery.pas');
    T:=P.Targets.AddImplicitUnit('jwamstask.pas');
    T:=P.Targets.AddImplicitUnit('jwamstcpip.pas');
    T:=P.Targets.AddImplicitUnit('jwamswsock.pas');
    T:=P.Targets.AddImplicitUnit('jwanative.pas');
    T:=P.Targets.AddImplicitUnit('jwanb30.pas');
    T:=P.Targets.AddImplicitUnit('jwanetsh.pas');
    T:=P.Targets.AddImplicitUnit('jwanspapi.pas');
    T:=P.Targets.AddImplicitUnit('jwantddpar.pas');
    T:=P.Targets.AddImplicitUnit('jwantdsapi.pas');
    T:=P.Targets.AddImplicitUnit('jwantdsbcli.pas');
    T:=P.Targets.AddImplicitUnit('jwantdsbmsg.pas');
    T:=P.Targets.AddImplicitUnit('jwantldap.pas');
    T:=P.Targets.AddImplicitUnit('jwantquery.pas');
    T:=P.Targets.AddImplicitUnit('jwantsecapi.pas');
    T:=P.Targets.AddImplicitUnit('jwantstatus.pas');
    T:=P.Targets.AddImplicitUnit('jwaobjsel.pas');
    T:=P.Targets.AddImplicitUnit('jwapatchapi.pas');
    T:=P.Targets.AddImplicitUnit('jwapatchwiz.pas');
    T:=P.Targets.AddImplicitUnit('jwapbt.pas');
    T:=P.Targets.AddImplicitUnit('jwapdhmsg.pas');
    T:=P.Targets.AddImplicitUnit('jwapdh.pas');
    T:=P.Targets.AddImplicitUnit('jwapowrprof.pas');
    T:=P.Targets.AddImplicitUnit('jwaprofinfo.pas');
    T:=P.Targets.AddImplicitUnit('jwaprotocol.pas');
    T:=P.Targets.AddImplicitUnit('jwaprsht.pas');
    T:=P.Targets.AddImplicitUnit('jwapsapi.pas');
    T:=P.Targets.AddImplicitUnit('jwaqosname.pas');
    T:=P.Targets.AddImplicitUnit('jwaqospol.pas');
    T:=P.Targets.AddImplicitUnit('jwaqos.pas');
    T:=P.Targets.AddImplicitUnit('jwaqossp.pas');
    T:=P.Targets.AddImplicitUnit('jwareason.pas');
    T:=P.Targets.AddImplicitUnit('jwaregstr.pas');
    T:=P.Targets.AddImplicitUnit('jwarpcasync.pas');
    T:=P.Targets.AddImplicitUnit('jwarpcdce.pas');
    T:=P.Targets.AddImplicitUnit('jwarpcnsi.pas');
    T:=P.Targets.AddImplicitUnit('jwarpcnterr.pas');
    T:=P.Targets.AddImplicitUnit('jwarpc.pas');
    T:=P.Targets.AddImplicitUnit('jwarpcssl.pas');
    T:=P.Targets.AddImplicitUnit('jwascesvc.pas');
    T:=P.Targets.AddImplicitUnit('jwaschedule.pas');
    T:=P.Targets.AddImplicitUnit('jwaschemadef.pas');
    T:=P.Targets.AddImplicitUnit('jwasddl.pas');
    T:=P.Targets.AddImplicitUnit('jwasecext.pas');
    T:=P.Targets.AddImplicitUnit('jwasecurity.pas');
    T:=P.Targets.AddImplicitUnit('jwasensapi.pas');
    T:=P.Targets.AddImplicitUnit('jwasensevts.pas');
    T:=P.Targets.AddImplicitUnit('jwasens.pas');
    T:=P.Targets.AddImplicitUnit('jwasfc.pas');
    T:=P.Targets.AddImplicitUnit('jwashlguid.pas');
    T:=P.Targets.AddImplicitUnit('jwasisbkup.pas');
    T:=P.Targets.AddImplicitUnit('jwasnmp.pas');
    T:=P.Targets.AddImplicitUnit('jwasporder.pas');
    T:=P.Targets.AddImplicitUnit('jwasrrestoreptapi.pas');
    T:=P.Targets.AddImplicitUnit('jwasspi.pas');
    T:=P.Targets.AddImplicitUnit('jwasubauth.pas');
    T:=P.Targets.AddImplicitUnit('jwasvcguid.pas');
    T:=P.Targets.AddImplicitUnit('jwatlhelp32.pas');
    T:=P.Targets.AddImplicitUnit('jwatmschema.pas');
    T:=P.Targets.AddImplicitUnit('jwatraffic.pas');
    T:=P.Targets.AddImplicitUnit('jwauserenv.pas');
    T:=P.Targets.AddImplicitUnit('jwauxtheme.pas');
    T:=P.Targets.AddImplicitUnit('jwavista.pas');
    T:=P.Targets.AddImplicitUnit('jwawbemcli.pas');
    T:=P.Targets.AddImplicitUnit('jwawinable.pas');
    T:=P.Targets.AddImplicitUnit('jwawinbase.pas');
    T:=P.Targets.AddImplicitUnit('jwawinber.pas');
    T:=P.Targets.AddImplicitUnit('jwawincon.pas');
    T:=P.Targets.AddImplicitUnit('jwawincpl.pas');
    T:=P.Targets.AddImplicitUnit('jwawincred.pas');
    T:=P.Targets.AddImplicitUnit('jwawincrypt.pas');
    T:=P.Targets.AddImplicitUnit('jwawindllnames.pas');
    T:=P.Targets.AddImplicitUnit('jwawindns.pas');
    T:=P.Targets.AddImplicitUnit('jwawindows.pas');
    T:=P.Targets.AddImplicitUnit('jwawinefs.pas');
    T:=P.Targets.AddImplicitUnit('jwawinerror.pas');
    T:=P.Targets.AddImplicitUnit('jwawinfax.pas');
    T:=P.Targets.AddImplicitUnit('jwawingdi.pas');
    T:=P.Targets.AddImplicitUnit('jwawinioctl.pas');
    T:=P.Targets.AddImplicitUnit('jwawinldap.pas');
    T:=P.Targets.AddImplicitUnit('jwawinnetwk.pas');
    T:=P.Targets.AddImplicitUnit('jwawinnls.pas');
    T:=P.Targets.AddImplicitUnit('jwawinnt.pas');
    T:=P.Targets.AddImplicitUnit('jwawinperf.pas');
    T:=P.Targets.AddImplicitUnit('jwawinreg.pas');
    T:=P.Targets.AddImplicitUnit('jwawinresrc.pas');
    T:=P.Targets.AddImplicitUnit('jwawinsafer.pas');
    T:=P.Targets.AddImplicitUnit('jwawinsock2.pas');
    T:=P.Targets.AddImplicitUnit('jwawinsock.pas');
    T:=P.Targets.AddImplicitUnit('jwawinsta.pas');
    T:=P.Targets.AddImplicitUnit('jwawinsvc.pas');
    T:=P.Targets.AddImplicitUnit('jwawinternl.pas');
    T:=P.Targets.AddImplicitUnit('jwawintype.pas');
    T:=P.Targets.AddImplicitUnit('jwawinuser.pas');
    T:=P.Targets.AddImplicitUnit('jwawinver.pas');
    T:=P.Targets.AddImplicitUnit('jwawinwlx.pas');
    T:=P.Targets.AddImplicitUnit('jwawmistr.pas');
    T:=P.Targets.AddImplicitUnit('jwawownt16.pas');
    T:=P.Targets.AddImplicitUnit('jwawownt32.pas');
    T:=P.Targets.AddImplicitUnit('jwawpapimsg.pas');
    T:=P.Targets.AddImplicitUnit('jwawpapi.pas');
    T:=P.Targets.AddImplicitUnit('jwawpcrsmsg.pas');
    T:=P.Targets.AddImplicitUnit('jwawpftpmsg.pas');
    T:=P.Targets.AddImplicitUnit('jwawppstmsg.pas');
    T:=P.Targets.AddImplicitUnit('jwawpspihlp.pas');
    T:=P.Targets.AddImplicitUnit('jwawptypes.pas');
    T:=P.Targets.AddImplicitUnit('jwawpwizmsg.pas');
    T:=P.Targets.AddImplicitUnit('jwaws2atm.pas');
    T:=P.Targets.AddImplicitUnit('jwaws2bth.pas');
    T:=P.Targets.AddImplicitUnit('jwaws2dnet.pas');
    T:=P.Targets.AddImplicitUnit('jwaws2spi.pas');
    T:=P.Targets.AddImplicitUnit('jwaws2tcpip.pas');
    T:=P.Targets.AddImplicitUnit('jwawshisotp.pas');
    T:=P.Targets.AddImplicitUnit('jwawsipx.pas');
    T:=P.Targets.AddImplicitUnit('jwawsnetbs.pas');
    T:=P.Targets.AddImplicitUnit('jwawsnwlink.pas');
    T:=P.Targets.AddImplicitUnit('jwawsrm.pas');
    T:=P.Targets.AddImplicitUnit('jwawsvns.pas');
    T:=P.Targets.AddImplicitUnit('jwawtsapi32.pas');
    T:=P.Targets.AddImplicitUnit('jwazmouse.pas');

    // Build unit depending on all implicit units
    TBuild:=P.Targets.AddUnit('buildjwa.pp');
      TBuild.Install:=False;
<<<<<<< HEAD
<<<<<<< HEAD
      TBuild.Dependencies.AddInclude('src/jediapilib.inc');
      TBuild.Dependencies.AddInclude('src/jedi.inc');
      TBuild.Dependencies.AddInclude('src/ModuleLoader.pas');
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      For I:=0 to P.Targets.Count-1 do
        begin
          T:=P.Targets.TargetItems[I];
          if T.TargetType=ttImplicitUnit then
            TBuild.Dependencies.AddUnit(T.Name);
        end;

{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}
