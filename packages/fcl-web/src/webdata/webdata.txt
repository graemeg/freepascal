FPC WebData architecture
========================

The aim of this set of components is to be able to easily send data 
to a webapplication, and to handle updates of this data, all in a
webserver module.

The following components are used

- TFPWebDataProvider
  The central component, forming a bridge between TDataset and web content.

- TCustomWebdataInputAdaptor
    A class that transforms the input of a web request to something that
    TFPWebDataProvider understands. Example implementations are provided 
    for ExtJS, XML and JSON.

- TWebdataInputAdaptor
   A descendent of TCustomWebdataInputAdaptor that allows to select the
   input format from a list of known formats.

- TCustomHTTPDataContentProducer
    This class produces the response for the webapplication. It is an
    abstract class: descendents need to be made for the various expected
    outputs. Example implementations are provided for ExtJS, XML and JSON.

- THTTPDataContentProducer
   A descendent of TCustomHTTPDataContentProducer that allows to select the
   output format from a list of known formats.

- TFPWebProviderDataModule
   A THTTPSessionDatamodule descendent that can be used to handle data
   requests from a webclient. It functions as a container for
   TFPWebDataProvider components, InputAdaptors and Content producers.

   A module is registered in the Lazarus IDE package under File/New.

Typically, one will do the following
 - Create a TFPWebProviderDataModule from the IDE.
 - Drop some dataset components on it, and set them up for use with some
   datasources
 - For each dataset, drop a TFPWebDataProvider component on the module, 
   and connect it to the datasource. The name of this component is exposed
   to the client.
 - Drop a suitable input adaptor.
 
The data can then typically be read through the URL:
baseurl/modulename/providername/read
Or updated through the URLs
baseurl/modulename/providername/update
baseurl/modulename/providername/create
baseurl/modulename/providername/delete
where baseurl is the base URL for the web-application.

Large applications: factory support
For large-scale applications with lots of different datasets, there is
support for registering dataproviders in a central factory system:

The WebDataProviderManager function returns an instance of
TFPWebDataProviderManager.

It must be used to register WebDataProvider names and classes:

Function RegisterProvider(Const AProviderName : String; AClass : TFPCustomWebDataProviderClass) : TWebDataProviderDef; overload;

The first form registers a class: an instance of this class will
be created by the factory whenever a provider of name AProviderName is
requested.

The TFPWebProviderDataModule class is aware of the WebDataProviderManager
factory class, and will look there for a TFPCustomWebDataProvider instance
if none is found in the webmodule instance itself and the
'UseProviderManager' property is 'True'.

The WebDataProviderManager factory can also Register a complete datamodule:

Procedure RegisterDatamodule(Const AClass : TDatamoduleClass);

This will register all WebDataProvider instances on the datamodule:
An instance will be created, all  TFPCustomWebDataProvider instances
will be registered with their component names. 
When a provider belonging to such a datamodule is requested, then
the module will be created, and the requested TFPCustomWebDataProvider 
instance is returned. 

A provider instance can be requested with the following factory methods:

Function GetProvider(Const ADef : TWebDataProviderDef; AOwner : TComponent;out AContainer : TComponent): TFPCustomWebDataProvider; 
Function GetProvider(Const AProviderName : String; AOwner : TComponent; Out AContainer : TComponent): TFPCustomWebDataProvider;

The result is the provider instance. All instances are created using a
container module: either this is the module class used in RegisterDatamodule
or a vanilla TDatamodule class. This instance is returned in AContainer.

The container must be freed by the caller.

In practise, this means that one creates a datamodule, drops some
TFPWebDataProvider instances on it, and adds the following call
to the initialization section of the unit:

WebDataProviderManager.RegisterDatamodule(TMyDataModule);

The TFPWebProviderDataModule that handles web requests will then be able
to handle requests for the TFPWebDataProvider instances on the datamodule.

Note that the RegisterDataModule routine will create an instance of the
datamodule to get a list of provider components (it uses the component.name
property). The WebDataProviderManager's 'registering' property will be set 
to true: this way one can avoid connecting to a database during registration.

The WebDataProviderManager also handles the registration of inputadataptors
and output contents producers.


