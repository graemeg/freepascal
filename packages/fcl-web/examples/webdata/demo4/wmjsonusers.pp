unit wmjsonusers;

{$mode objfpc}{$H+}

interface

uses
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Classes, SysUtils, HTTPDefs, websession, fpHTTP, fpWeb, 
=======
  Classes, SysUtils, FileUtil, LResources, HTTPDefs, websession, fpHTTP, fpWeb, 
>>>>>>> graemeg/cpstrnew
=======
  Classes, SysUtils, FileUtil, LResources, HTTPDefs, websession, fpHTTP, fpWeb, 
>>>>>>> graemeg/cpstrnew
=======
  Classes, SysUtils, FileUtil, LResources, HTTPDefs, websession, fpHTTP, fpWeb, 
>>>>>>> graemeg/cpstrnew
=======
  Classes, SysUtils, FileUtil, LResources, HTTPDefs, websession, fpHTTP, fpWeb, 
>>>>>>> origin/cpstrnew
    fpwebdata, extjsjson;

type

  { TFPWebProviderDataModule1 }

  TFPWebProviderDataModule1 = class(TFPWebProviderDataModule)
    ExtJSJSONDataFormatter1: TExtJSJSONDataFormatter;
    ExtJSJSonWebdataInputAdaptor1: TExtJSJSonWebdataInputAdaptor;
  private
    { private declarations }
  public
    { public declarations }
  end; 

var
  FPWebProviderDataModule1: TFPWebProviderDataModule1; 

implementation

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$R *.lfm}

initialization
=======
initialization
  {$I wmjsonusers.lrs}

>>>>>>> graemeg/cpstrnew
=======
initialization
  {$I wmjsonusers.lrs}

>>>>>>> graemeg/cpstrnew
=======
initialization
  {$I wmjsonusers.lrs}

>>>>>>> graemeg/cpstrnew
=======
initialization
  {$I wmjsonusers.lrs}

>>>>>>> origin/cpstrnew
  RegisterHTTPModule('JSONProvider', TFPWebProviderDataModule1);
end.

