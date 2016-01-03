This is the beginning of a server side web system for FPC.
Although it is non-visual, it is geared towards use in Lazarus.

Architecture:

httpdefs
--------
contains the basic HTTP system definitions: 
header field names
TCookie(s): 
  collection with cookies in web request
TUploadedFile(s): 
  collection with uploaded files in request
THTTPHeader:  
  Class describes HTTP Request/Response headers.
  it has properties for all possible HTTP headers, including cookies
TRequest: 
  Descendent of THTTPHeader, describes client request.
  Contains uploaded files.
TResponse: 
  describes the web server response. Includes headers and contents. 
TCustomSession: 
 Base for all session components.

fphttp:
-------
Basic web system components/classes

TCustomHTTPModule: 
  Abstract TDataModule descendant, which processes a webrequest
  and prepares a response.
TModuleFactory/TModuleItem: 
  Module registration and creation system.
TCustomWebAction(s): 
  Webactions.
RegisterHTTPModule():
  routine registers a module with the web system.
THTTPContentProducer : 
  abstract HTTP producer component.

The idea is that the URL relative to the server is taken and parsed as
follows
http://www.server.org/Path1/Path2

Path1 determines which module in the system is created to handle the
request. (the factory is queried)
Path2 determines which web action inside the module is used to handle the
request. (implemented in TWebModule, see fpweb)

websession
----------
Implements basic session support.

TSessionHTTPModule:  
  TCustomHTTPModule descendent with session support.
TIniWebSession: 
  TCustomSession descendent which stores session variables in inifiles.
TFPWebSession:  
  TIniWebSession descendent for use in fpweb. Uses cookies to store session info.
GetDefaultSession() : 
  returns default session object.

fptemplate
----------
Unit which implements template support.

TTemplateParser: 
  Template parse object. Does the actual parsing and replacing. Delimiters
  are configurable. Standard variables can be specified, callbacks for unknown
  values can be set.

TFPCustomTemplate:
  TPersistent for use in components. Allows properties to configure the
  TTemplateParser. Supports streams, template and template files.

fpweb
-----
Actual usable implementation of TCustomHTTPModule. 

TFPWebAction(s): 
  Web actions with template support.
TCustomFPWebModule: 
  Descends from TSessionHTTPModule, implements WebActions and has template support.
TFPWebModule: 
  published TCustomFPWebModule properties for use in Lazarus.

fphtml
------
This creates web modules specialized in creating HTML content.

THTMLContentProducer: 
  Descendent of THTTPContentProducer which produces HTML
THTMLCustomDatasetContentProducer: 
  Descendent of THTTPContentProducer which produces HTML from datasets.
THTMLDatasetContentProducer: 
  Descendent of THTMLCustomDatasetContentProducer which publishes some 
  properties.
THTMLSelectProducer:
  Produces a combo box.
THTMLDatasetSelectProducer
  Produces a combo box bases on values in a dataset.
TCustomHTMLModule:
  TCustomHTTPModule descendent which produces HTML content only.

Note that classes for HTML document creation come from package fcl-xml.

fpdatasetform
-------------

This contains classes which allow to create complicated HTML/forms 
based on a TDataset.

THTMLDatasetFormProducer
  Creates an edit form for a TDataset record.
  Complicated table layouts are possible.

THTMLDatasetFormGridProducer
  Creates a grid with data from a TDataset
  Complicated table formatting is possible.

custcgi:
--------
  CGI application base class. It knows nothing of the fp
  HTTP module system.

TCustomCGIApplication : 
  TCustomApplication descendent which handles a CGI request.
  No instance of this class is created, this is done in fpcgi.

TCGIRequest:
  TRequest descendent which retrieves content from the CGI 
  environment.

TCGIResponse:
  TResponse descendent which returns content to the CGI environment.

fpcgi:
------
  Standard CGI application instance.

TCGIApplication:
  TCustomCGIApplication descendent which uses the fpWeb system
  to create a TCustomHTTPModuleClass to handle the request.

It contains an Application instance which handles everything.

fpApache:
---------
  Apache Module support.

TCustomApacheApplication:
  Uses fpweb to create TCustomHTTPModuleClass instances to 
  handle the request. 

