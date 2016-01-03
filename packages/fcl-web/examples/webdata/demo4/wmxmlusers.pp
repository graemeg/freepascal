unit wmxmlusers; 

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
    fpwebdata, extjsxml;

type

  { TFPWebProviderDataModule2 }

  TFPWebProviderDataModule2 = class(TFPWebProviderDataModule)
    ExtJSXMLDataFormatter1: TExtJSXMLDataFormatter;
    ExtJSXMLWebdataInputAdaptor1: TExtJSXMLWebdataInputAdaptor;
  private
    { private declarations }
  public
    { public declarations }
  end; 

var
  FPWebProviderDataModule2: TFPWebProviderDataModule2; 

implementation

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$R *.lfm}

initialization
=======
initialization
  {$I wmxmlusers.lrs}

>>>>>>> graemeg/cpstrnew
=======
initialization
  {$I wmxmlusers.lrs}

>>>>>>> graemeg/cpstrnew
=======
initialization
  {$I wmxmlusers.lrs}

>>>>>>> graemeg/cpstrnew
=======
initialization
  {$I wmxmlusers.lrs}

>>>>>>> origin/cpstrnew
  RegisterHTTPModule('XMLProvider', TFPWebProviderDataModule2);
end.

