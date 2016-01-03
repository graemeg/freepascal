{ Parsed from Foundation.framework NSConnection.h }

{$ifdef TYPES}
{$ifndef NSCONNECTION_PAS_T}
{$define NSCONNECTION_PAS_T}

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSCONNECTION_PAS_R}
{$define NSCONNECTION_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSCONNECTION_PAS_F}
{$define NSCONNECTION_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSCONNECTION_PAS_S}
{$define NSCONNECTION_PAS_S}

{ External string constants }
var
  NSConnectionReplyMode: NSString; cvar; external;
  NSConnectionDidDieNotification: NSString; cvar; external;
  NSFailedAuthenticationException: NSString; cvar; external;
  NSConnectionDidInitializeNotification: NSString; cvar; external;

{$endif}
{$endif}

{$ifdef FORWARD}
  NSConnectionDelegateProtocol = objcprotocol;
  NSConnection = objcclass;
  NSConnectionPointer = ^NSConnection;
  NSConnectionPtr = NSConnectionPointer;
  NSDistantObjectRequest = objcclass;
  NSDistantObjectRequestPointer = ^NSDistantObjectRequest;
  NSDistantObjectRequestPtr = NSDistantObjectRequestPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSCONNECTION_PAS_C}
{$define NSCONNECTION_PAS_C}

{ NSConnection }
  NSConnection = objcclass external (NSObject)
  private
    _receivePort: id;
    _sendPort: id;
    _delegate: id;
    busy: cint32;
    localProxyCount: cint32;
    waitCount: cint32;
    delayedRL: id;
    _statistics: id;
    isDead: char;
    _isValid: char;
    wantsInvalid: char;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    _anoninternstruct_NSConnection0: record
      case byte of
       0: (_anonbitfield_NSConnection0: char);
       1: (data: bitpacked record
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    _anoninternstruct_NSConnection0: bitpacked record
      case byte of
       0: (_anonbitfield_NSConnection0: char);
       1: (
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
        authGen: 0..1;
        authCheck: 0..1;
        encryptFlag: 0..1;
        decryptFlag: 0..1;
        doRequest: 0..1;
        isQueueing: 0..1;
        isMulti: 0..1;
        invalidateRP: 0..1;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
       end;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
      );
      end;
    ___1: id;
    ___2: id;
    runLoops: id;
    _requestModes: id;
    _rootObject: id;
    registerInfo: Pointer;
    replMode: id;
    classInfoImported: id;
    releasedProxies: id;
    reserved: Pointer;
    
  public
    function statistics: NSDictionary; message 'statistics';
    class function allConnections: NSArray; message 'allConnections';
    class function defaultConnection: NSConnection; message 'defaultConnection'; deprecated 'in Mac OS X 10.6 and later';
    class function connectionWithRegisteredName_host(name: NSString; hostName: NSString): id; message 'connectionWithRegisteredName:host:';
    class function connectionWithRegisteredName_host_usingNameServer(name: NSString; hostName: NSString; server: NSPortNameServer): id; message 'connectionWithRegisteredName:host:usingNameServer:';
    class function rootProxyForConnectionWithRegisteredName_host(name: NSString; hostName: NSString): NSDistantObject; message 'rootProxyForConnectionWithRegisteredName:host:';
    class function rootProxyForConnectionWithRegisteredName_host_usingNameServer(name: NSString; hostName: NSString; server: NSPortNameServer): NSDistantObject; message 'rootProxyForConnectionWithRegisteredName:host:usingNameServer:';
    class function serviceConnectionWithName_rootObject_usingNameServer(name: NSString; root: id; server: NSPortNameServer): id; message 'serviceConnectionWithName:rootObject:usingNameServer:';
    class function serviceConnectionWithName_rootObject(name: NSString; root: id): id; message 'serviceConnectionWithName:rootObject:';
    procedure setRequestTimeout(ti: NSTimeInterval); message 'setRequestTimeout:';
    function requestTimeout: NSTimeInterval; message 'requestTimeout';
    procedure setReplyTimeout(ti: NSTimeInterval); message 'setReplyTimeout:';
    function replyTimeout: NSTimeInterval; message 'replyTimeout';
    procedure setRootObject(anObject: id); message 'setRootObject:';
    function rootObject: id; message 'rootObject';
    function rootProxy: NSDistantObject; message 'rootProxy';
    procedure setDelegate(anObject: NSConnectionDelegateProtocol); message 'setDelegate:';
    function delegate: NSConnectionDelegateProtocol; message 'delegate';
    procedure setIndependentConversationQueueing(yorn: Boolean); message 'setIndependentConversationQueueing:';
    function independentConversationQueueing: Boolean; message 'independentConversationQueueing';
    function isValid: Boolean; message 'isValid';
    procedure invalidate; message 'invalidate';
    procedure addRequestMode(rmode: NSString); message 'addRequestMode:';
    procedure removeRequestMode(rmode: NSString); message 'removeRequestMode:';
    function requestModes: NSArray; message 'requestModes';
    function registerName(name: NSString): Boolean; message 'registerName:';
    function registerName_withNameServer(name: NSString; server: NSPortNameServer): Boolean; message 'registerName:withNameServer:';
    class function connectionWithReceivePort_sendPort(receivePort_: NSPort; sendPort_: NSPort): id; message 'connectionWithReceivePort:sendPort:';
    class function currentConversation: id; message 'currentConversation';
    function initWithReceivePort_sendPort(receivePort_: NSPort; sendPort_: NSPort): id; message 'initWithReceivePort:sendPort:';
    function sendPort: NSPort; message 'sendPort';
    function receivePort: NSPort; message 'receivePort';
    procedure enableMultipleThreads; message 'enableMultipleThreads';
    function multipleThreadsEnabled: Boolean; message 'multipleThreadsEnabled';
    procedure addRunLoop(runloop: NSRunLoop); message 'addRunLoop:';
    procedure removeRunLoop(runloop: NSRunLoop); message 'removeRunLoop:';
    procedure runInNewThread; message 'runInNewThread';
    function remoteObjects: NSArray; message 'remoteObjects';
    function localObjects: NSArray; message 'localObjects';
  end;

{ NSDistantObjectRequest }
  NSDistantObjectRequest = objcclass external (NSObject)
    
  public
    function invocation: NSInvocation; message 'invocation';
    function connection: NSConnection; message 'connection';
    function conversation: id; message 'conversation';
    procedure replyWithException(exception: NSException); message 'replyWithException:';
  end;

{$endif}
{$endif}
{$ifdef PROTOCOLS}
{$ifndef NSCONNECTION_PAS_P}
{$define NSCONNECTION_PAS_P}
  
{ NSConnectionDelegate Protocol }
  NSConnectionDelegateProtocol = objcprotocol external name 'NSConnectionDelegate'
  optional
    function makeNewConnection_sender(conn: NSConnection; ancestor: NSConnection): Boolean; message 'makeNewConnection:sender:';
    function connection_shouldMakeNewConnection(ancestor: NSConnection; conn: NSConnection): Boolean; message 'connection:shouldMakeNewConnection:';
    function authenticationDataForComponents(components: NSArray): NSData; message 'authenticationDataForComponents:';
    function authenticateComponents_withData(components: NSArray; signature: NSData): Boolean; message 'authenticateComponents:withData:';
    function createConversationForConnection(conn: NSConnection): id; message 'createConversationForConnection:';
    function connection_handleRequest(connection: NSConnection; doreq: NSDistantObjectRequest): Boolean; message 'connection:handleRequest:';
  end;
{$endif}
{$endif}
