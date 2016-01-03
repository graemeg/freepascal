{
     File:       CarbonCore/FSEvents.h
 
     Contains:   FSEventStream API
 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     Copyright:  © 2006-2011 by Apple Inc. All rights reserved.
=======
     Version:    CarbonCore-859.2~1
 
     Copyright:  © 2006-2008 by Apple Computer, Inc.  All rights reserved
>>>>>>> graemeg/cpstrnew
=======
     Version:    CarbonCore-859.2~1
 
     Copyright:  © 2006-2008 by Apple Computer, Inc.  All rights reserved
>>>>>>> graemeg/cpstrnew
=======
     Version:    CarbonCore-859.2~1
 
     Copyright:  © 2006-2008 by Apple Computer, Inc.  All rights reserved
>>>>>>> graemeg/cpstrnew
=======
     Version:    CarbonCore-859.2~1
 
     Copyright:  © 2006-2008 by Apple Computer, Inc.  All rights reserved
>>>>>>> origin/cpstrnew
=======
     Version:    CarbonCore-859.2~1
 
     Copyright:  © 2006-2008 by Apple Computer, Inc.  All rights reserved
>>>>>>> origin/fixes_2.4
 
     Bugs?:      For bug reports, consult the following page on
                 the World Wide Web:
 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                     http://bugs.freepascal.org
 
}
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
                     http://www.freepascal.org/bugs.html
 
}
{   Pascal Translation:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
<<<<<<< HEAD
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
=======
>>>>>>> origin/fixes_2.4
{
    Modified for use with Free Pascal
    Version 308
    Please report any bugs to <gpc@microbizz.nl>
}

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}
{$mode macpas}
{$packenum 1}
{$macro on}
{$inline on}
{$calling mwpascal}

unit FSEvents;
interface
{$setc UNIVERSAL_INTERFACES_VERSION := $0400}
{$setc GAP_INTERFACES_VERSION := $0308}

{$ifc not defined USE_CFSTR_CONSTANT_MACROS}
    {$setc USE_CFSTR_CONSTANT_MACROS := TRUE}
{$endc}

{$ifc defined CPUPOWERPC and defined CPUI386}
	{$error Conflicting initial definitions for CPUPOWERPC and CPUI386}
{$endc}
{$ifc defined FPC_BIG_ENDIAN and defined FPC_LITTLE_ENDIAN}
	{$error Conflicting initial definitions for FPC_BIG_ENDIAN and FPC_LITTLE_ENDIAN}
{$endc}

{$ifc not defined __ppc__ and defined CPUPOWERPC32}
	{$setc __ppc__ := 1}
{$elsec}
	{$setc __ppc__ := 0}
{$endc}
{$ifc not defined __ppc64__ and defined CPUPOWERPC64}
	{$setc __ppc64__ := 1}
{$elsec}
	{$setc __ppc64__ := 0}
{$endc}
{$ifc not defined __i386__ and defined CPUI386}
	{$setc __i386__ := 1}
{$elsec}
	{$setc __i386__ := 0}
{$endc}
{$ifc not defined __x86_64__ and defined CPUX86_64}
	{$setc __x86_64__ := 1}
{$elsec}
	{$setc __x86_64__ := 0}
{$endc}
{$ifc not defined __arm__ and defined CPUARM}
	{$setc __arm__ := 1}
{$elsec}
	{$setc __arm__ := 0}
{$endc}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifc not defined __arm64__ and defined CPUAARCH64}
  {$setc __arm64__ := 1}
{$elsec}
  {$setc __arm64__ := 0}
{$endc}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4

{$ifc defined cpu64}
  {$setc __LP64__ := 1}
{$elsec}
  {$setc __LP64__ := 0}
{$endc}


{$ifc defined __ppc__ and __ppc__ and defined __i386__ and __i386__}
	{$error Conflicting definitions for __ppc__ and __i386__}
{$endc}

{$ifc defined __ppc__ and __ppc__}
	{$setc TARGET_CPU_PPC := TRUE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	{$setc TARGET_CPU_ARM64 := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
	{$setc TARGET_OS_EMBEDDED := FALSE}
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> graemeg/cpstrnew
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> graemeg/cpstrnew
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> graemeg/cpstrnew
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> origin/cpstrnew
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> origin/fixes_2.4
{$elifc defined __ppc64__ and __ppc64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := TRUE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	{$setc TARGET_CPU_ARM64 := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
	{$setc TARGET_OS_EMBEDDED := FALSE}
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> graemeg/cpstrnew
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> graemeg/cpstrnew
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> graemeg/cpstrnew
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> origin/cpstrnew
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> origin/fixes_2.4
{$elifc defined __i386__ and __i386__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := TRUE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	{$setc TARGET_CPU_ARM64 := FALSE}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
{$ifc defined(iphonesim)}
 	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := TRUE}
{$elsec}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$endc}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	{$setc TARGET_OS_EMBEDDED := FALSE}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
{$elifc defined __x86_64__ and __x86_64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := TRUE}
	{$setc TARGET_CPU_ARM := FALSE}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	{$setc TARGET_CPU_ARM64 := FALSE}
{$ifc defined(iphonesim)}
 	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := TRUE}
{$elsec}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$endc}
	{$setc TARGET_OS_EMBEDDED := FALSE}
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> graemeg/cpstrnew
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> graemeg/cpstrnew
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> graemeg/cpstrnew
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> origin/cpstrnew
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> origin/fixes_2.4
{$elifc defined __arm__ and __arm__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := TRUE}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	{$setc TARGET_CPU_ARM64 := FALSE}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
	{ will require compiler define when/if other Apple devices with ARM cpus ship }
	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	{$setc TARGET_OS_EMBEDDED := TRUE}
{$elifc defined __arm64__ and __arm64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_CPU_ARM64 := TRUE}
=======
>>>>>>> graemeg/cpstrnew
	{ will require compiler define when/if other Apple devices with ARM cpus ship }
	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
<<<<<<< HEAD
	{$setc TARGET_OS_EMBEDDED := TRUE}
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ nor __arm64__ is defined.}
=======
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
>>>>>>> graemeg/cpstrnew
=======
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
>>>>>>> graemeg/cpstrnew
=======
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
>>>>>>> graemeg/cpstrnew
=======
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
>>>>>>> origin/cpstrnew
=======
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
>>>>>>> origin/fixes_2.4
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
  {$setc TARGET_CPU_64 := FALSE}
{$endc}

{$ifc defined FPC_BIG_ENDIAN}
	{$setc TARGET_RT_BIG_ENDIAN := TRUE}
	{$setc TARGET_RT_LITTLE_ENDIAN := FALSE}
{$elifc defined FPC_LITTLE_ENDIAN}
	{$setc TARGET_RT_BIG_ENDIAN := FALSE}
	{$setc TARGET_RT_LITTLE_ENDIAN := TRUE}
{$elsec}
	{$error Neither FPC_BIG_ENDIAN nor FPC_LITTLE_ENDIAN are defined.}
{$endc}
{$setc ACCESSOR_CALLS_ARE_FUNCTIONS := TRUE}
{$setc CALL_NOT_IN_CARBON := FALSE}
{$setc OLDROUTINENAMES := FALSE}
{$setc OPAQUE_TOOLBOX_STRUCTS := TRUE}
{$setc OPAQUE_UPP_TYPES := TRUE}
{$setc OTCARBONAPPLICATION := TRUE}
{$setc OTKERNEL := FALSE}
{$setc PM_USE_SESSION_APIS := TRUE}
{$setc TARGET_API_MAC_CARBON := TRUE}
{$setc TARGET_API_MAC_OS8 := FALSE}
{$setc TARGET_API_MAC_OSX := TRUE}
{$setc TARGET_CARBON := TRUE}
{$setc TARGET_CPU_68K := FALSE}
{$setc TARGET_CPU_MIPS := FALSE}
{$setc TARGET_CPU_SPARC := FALSE}
{$setc TARGET_OS_UNIX := FALSE}
{$setc TARGET_OS_WIN32 := FALSE}
{$setc TARGET_RT_MAC_68881 := FALSE}
{$setc TARGET_RT_MAC_CFM := FALSE}
{$setc TARGET_RT_MAC_MACHO := TRUE}
{$setc TYPED_FUNCTION_POINTERS := TRUE}
{$setc TYPE_BOOL := FALSE}
{$setc TYPE_EXTENDED := FALSE}
{$setc TYPE_LONGLONG := TRUE}
uses MacTypes,MacOSXPosix,CFBase,CFArray,CFDate,CFRunLoop,CFUUID;
{$endc} {not MACOSALLINCLUDE}


{$ifc TARGET_OS_MAC}

{$ALIGN MAC68K}


{
 *  
 *  
 *  Discussion:
 *    This header describes the FSEvents API.  This API provides a
 *    mechanism to notify clients about directories they ought to
 *    re-scan in order to keep their internal data structures
 *    up-to-date with respect to the true state of the file system.
 *    (For example, when files or directories are created, modified, or
 *    removed.) It sends these notifications "in bulk", possibly
 *    notifying the client of changes to several directories in a
 *    single callback. By using the API, clients can notice such
 *    changes quickly, without needing to resort to recursive
 *    polling/scanning of the file system. 
 *    
 *    Much like kqueues, the FSEvents API allows an application to find
 *    near-immediately when the contents of a particular directory has
 *    changed.  However, unlike kqueues, the FSEvents API allows the
 *    application to monitor the whole file system hierarchy rooted at
 *    a specified directory (and still get precise per-directory
 *    notifications) -- to do this with the kqueues API would require
 *    the client to monitor each directory individually. 
 *    
 *    Clients can register interest in a chunk of the filesystem
 *    hierarchy and will receive callbacks from their runloop whenever
 *    an event occurs that modifies the filesystem therein.  The
 *    callback will indicate the exact directory in which the event
 *    occurred, so the client only has to scan that directory for
 *    updated info, not all its children.  Clients can supply a
 *    "latency" parameter that tells how long to wait after an event
 *    occurs before forwarding it; this reduces the volume of events
 *    and reduces the chance that the client will see an "intermediate"
 *    state, like those that arise when doing a "safe save" of a file,
 *    creating a package, or downloading a file via Safari. 
 *    
 *    
 *    
 *    The lifecycle of an FSEventStream consists of these stages:
 *    
 *    
 *    1. FSEventStreamCreate() / FSEventStreamCreateRelativeToDevice()
 *    -> Creates an FSEventStream. 
 *    
 *    2. FSEventStreamScheduleWithRunLoop() -> Schedules an
 *    FSEventStream on a runloop, like CFRunLoopAddSource() does for a
 *    CFRunLoopSourceRef. 
 *    
 *    3. FSEventStreamStart() -> Starts receiving events and servicing
 *    them from the client's runloop(s) using the callback supplied by
 *    the client when the stream was created. If a value was supplied
 *    for the sinceWhen parameter then "historical" events will be sent
 *    via your callback first, then a HistoryDone event, then
 *    "contemporary" events will be sent on an ongoing basis (as though
 *    you had supplied kFSEventStreamEventIdSinceNow for sinceWhen).
 *    
 *    
 *    4. FSEventStreamStop()  -> Stops the stream, ensuring the
 *    client's callback will not be called again for this stream.   
 *    After stopping the stream, it can be restarted seamlessly via
 *    FSEventStreamStart() without missing any events. 
 *    
 *    5. FSEventStreamInvalidate() -> Invalidates the stream, like
 *    CFRunLoopSourceInvalidate() does for a CFRunLoopSourcRef.
 *    
 *    
 *    6. FSEventStreamRelease() -> Decrements the refcount on the
 *    stream (initially one and incremented via FSEventStreamRetain()).
 *     If the refcount reaches zero, the stream is deallocated.
 *    
 *    
 *    Once the event stream has been started, the following calls can
 *    be used: 
 *    
 *    FSEventStreamGetLatestEventId() -> Initially, this returns the
 *    sinceWhen value supplied when the stream was created; thereafter,
 *    it is updated with the highest-numbered event ID mentioned in the
 *    current batch of events just before invoking the client's
 *    callback. Clients can store this value persistently as long as
 *    they also store the UUID for the device (obtained via
 *    FSEventsCopyUUIDForDevice()).  Clients can then later supply this
 *    event ID as the sinceWhen parameter to
 *    FSEventStreamCreateRelativeToDevice(), as long as its UUID
 *    matches what you stored.  This works because the FSEvents service
 *    stores events in a persistent, per-volume database.  In this
 *    regard,the stream of event IDs acts like a global, system-wide
 *    clock, but bears no relation to any particular timebase. 
 *    
 *    FSEventStreamFlushAsync() -> Requests that the fseventsd daemon
 *    send any events it has already buffered (via the latency
 *    parameter to one of the FSEventStreamCreate...() functions). This
 *    occurs asynchronously; clients will not have received all the
 *    callbacks by the time this call returns to them. 
 *    
 *    FSEventStreamFlushSync() -> Requests that the fseventsd daemon
 *    send any events it has already buffered (via the latency
 *    parameter to one of the FSEventStreamCreate...() functions). Then
 *    runs the runloop in its private mode till all events that have
 *    occurred have been reported (via the clients callback).  This
 *    occurs synchronously; clients will have received all the
 *    callbacks by the time this call returns to them. 
 *    
 *    FSEventStreamGetDeviceBeingWatched() -> Gets the dev_t value
 *    supplied when the stream was created with
 *    FSEventStreamCreateRelativeToDevice(), otherwise 0. 
 *    
 *    FSEventStreamCopyPathsBeingWatched() -> Gets the paths supplied
 *    when the stream was created with one of the
 *    FSEventStreamCreate...() functions. 
 *    
 *    Calls that can be made without a stream: 
 *    
 *    FSEventsCopyUUIDForDevice() -> Gets a UUID that uniquely
 *    identifies the FSEvents database for that volume. If the database
 *    gets discarded then its replacement will have a different UUID so
 *    that clients will be able to detect this situation and avoid
 *    trying to use event IDs that they stored as the sinceWhen
 *    parameter to the FSEventStreamCreate...() functions. 
 *    
 *    FSEventsGetCurrentEventId() -> Gets the most recently generated
 *    event ID, system-wide (not just for one stream). 
 *    
 *    FSEventsGetLastEventIdForDeviceBeforeTime() -> Gets the last
 *    event ID for the given device that was returned before the given
 *    time.  This is conservative in the sense that if you then use the
 *    returned event ID as the sinceWhen parameter of
 *    FSEventStreamCreateRelativeToDevice() that you will not miss any
 *    events that happened since that time.  On the other hand, you
 *    might receive some (harmless) extra events. 
 *    
 *    FSEventsPurgeEventsForDeviceUpToEventId() -> Purges old events
 *    from the persistent per-volume database maintained by the
 *    service.  You can combine this with
 *    FSEventsGetLastEventIdForDeviceBeforeTime(). Can only be called
 *    by the root user.
 }
{
 * Types and Constants
 }

{
 *  FSEventStreamCreateFlags
 *  
 *  Discussion:
 *    Flags that can be passed to the FSEventStreamCreate...()
 *    functions to modify the behavior of the stream being created.
 }
type
	FSEventStreamCreateFlags = UInt32;
	FSEventStreamCreateFlagsPtr = ^FSEventStreamCreateFlags;

{
 *  FSEventStreamCreateFlags
 *  
 *  Discussion:
 *    Flags that can be passed to the FSEventStreamCreate...()
 *    functions to modify the behavior of the stream being created.
 }
const
{
   * The default.
   }
	kFSEventStreamCreateFlagNone = $00000000;

  {
   * The framework will invoke your callback function with CF types
   * rather than raw C types (i.e., a CFArrayRef of CFStringRefs,
   * rather than a raw C array of raw C string pointers). See
   * FSEventStreamCallback.
   }
	kFSEventStreamCreateFlagUseCFTypes = $00000001;

  {
   * Affects the meaning of the latency parameter. If you specify this
   * flag and more than latency seconds have elapsed since the last
   * event, your app will receive the event immediately. The delivery
   * of the event resets the latency timer and any further events will
   * be delivered after latency seconds have elapsed. This flag is
   * useful for apps that are interactive and want to react immediately
   * to changes but avoid getting swamped by notifications when changes
   * are occurringin rapid succession. If you do not specify this flag,
   * then when an event occurs after a period of no events, the latency
   * timer is started. Any events that occur during the next latency
   * seconds will be delivered as one group (including that first
   * event). The delivery of the group of events resets the latency
   * timer and any further events will be delivered after latency
   * seconds. This is the default behavior and is more appropriate for
   * background, daemon or batch processing apps.
   }
	kFSEventStreamCreateFlagNoDefer = $00000002;

  {
   * Request notifications of changes along the path to the path(s)
   * you're watching. For example, with this flag, if you watch
   * "/foo/bar" and it is renamed to "/foo/bar.old", you would receive
   * a RootChanged event. The same is true if the directory "/foo" were
   * renamed. The event you receive is a special event: the path for
   * the event is the original path you specified, the flag
   * kFSEventStreamEventFlagRootChanged is set and event ID is zero.
   * RootChanged events are useful to indicate that you should rescan a
   * particular hierarchy because it changed completely (as opposed to
   * the things inside of it changing). If you want to track the
   * current location of a directory, it is best to open the directory
   * before creating the stream so that you have a file descriptor for
   * it and can issue an F_GETPATH fcntl() to find the current path.
   }
	kFSEventStreamCreateFlagWatchRoot = $00000004;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

  {
   * Don't send events that were triggered by the current process. This
   * is useful for reducing the volume of events that are sent. It is
   * only useful if your process might modify the file system hierarchy
   * beneath the path(s) being monitored. Note: this has no effect on
   * historical events, i.e., those delivered before the HistoryDone
   * sentinel event.
   }
	kFSEventStreamCreateFlagIgnoreSelf = $00000008;

  {
   * Request file-level notifications.  Your stream will receive events
   * about individual files in the hierarchy you're watching instead of
   * only receiving directory level notifications.  Use this flag with
   * care as it will generate significantly more events than without it.
   }
	kFSEventStreamCreateFlagFileEvents = $00000010;
	kFSEventStreamCreateFlagMarkSelf = $00000020;

=======
	kFSEventStreamCreateFlagIgnoreSelf = $00000008;

>>>>>>> graemeg/cpstrnew
=======
	kFSEventStreamCreateFlagIgnoreSelf = $00000008;

>>>>>>> graemeg/cpstrnew
=======
	kFSEventStreamCreateFlagIgnoreSelf = $00000008;

>>>>>>> graemeg/cpstrnew
=======
	kFSEventStreamCreateFlagIgnoreSelf = $00000008;

>>>>>>> origin/cpstrnew
=======
	kFSEventStreamCreateFlagIgnoreSelf = $00000008;

>>>>>>> origin/fixes_2.4

{
 *  FSEventStreamEventFlags
 *  
 *  Discussion:
 *    Flags that can be passed your FSEventStreamCallback function.
 }
type
	FSEventStreamEventFlags = UInt32;
	FSEventStreamEventFlagsPtr = ^FSEventStreamEventFlags;

{
 *  FSEventStreamEventFlags
 *  
 *  Discussion:
 *    Flags that can be passed to your FSEventStreamCallback function.
 }
const
{
   * There was some change in the directory at the specific path
   * supplied in this event.
   }
	kFSEventStreamEventFlagNone = $00000000;

  {
   * Your application must rescan not just the directory given in the
   * event, but all its children, recursively. This can happen if there
   * was a problem whereby events were coalesced hierarchically. For
   * example, an event in /Users/jsmith/Music and an event in
   * /Users/jsmith/Pictures might be coalesced into an event with this
   * flag set and path=/Users/jsmith. If this flag is set you may be
   * able to get an idea of whether the bottleneck happened in the
   * kernel (less likely) or in your client (more likely) by checking
   * for the presence of the informational flags
   * kFSEventStreamEventFlagUserDropped or
   * kFSEventStreamEventFlagKernelDropped.
   }
	kFSEventStreamEventFlagMustScanSubDirs = $00000001;

  {
   * The kFSEventStreamEventFlagUserDropped or
   * kFSEventStreamEventFlagKernelDropped flags may be set in addition
   * to the kFSEventStreamEventFlagMustScanSubDirs flag to indicate
   * that a problem occurred in buffering the events (the particular
   * flag set indicates where the problem occurred) and that the client
   * must do a full scan of any directories (and their subdirectories,
   * recursively) being monitored by this stream. If you asked to
   * monitor multiple paths with this stream then you will be notified
   * about all of them. Your code need only check for the
   * kFSEventStreamEventFlagMustScanSubDirs flag; these flags (if
   * present) only provide information to help you diagnose the problem.
   }
	kFSEventStreamEventFlagUserDropped = $00000002;
	kFSEventStreamEventFlagKernelDropped = $00000004;

  {
   * If kFSEventStreamEventFlagEventIdsWrapped is set, it means the
   * 64-bit event ID counter wrapped around. As a result,
   * previously-issued event ID's are no longer valid arguments for the
   * sinceWhen parameter of the FSEventStreamCreate...() functions.
   }
	kFSEventStreamEventFlagEventIdsWrapped = $00000008;

  {
   * Denotes a sentinel event sent to mark the end of the "historical"
   * events sent as a result of specifying a sinceWhen value in the
   * FSEventStreamCreate...() call that created this event stream. (It
   * will not be sent if kFSEventStreamEventIdSinceNow was passed for
   * sinceWhen.) After invoking the client's callback with all the
   * "historical" events that occurred before now, the client's
   * callback will be invoked with an event where the
   * kFSEventStreamEventFlagHistoryDone flag is set. The client should
   * ignore the path supplied in this callback.
   }
	kFSEventStreamEventFlagHistoryDone = $00000010;

  {
   * Denotes a special event sent when there is a change to one of the
   * directories along the path to one of the directories you asked to
   * watch. When this flag is set, the event ID is zero and the path
   * corresponds to one of the paths you asked to watch (specifically,
   * the one that changed). The path may no longer exist because it or
   * one of its parents was deleted or renamed. Events with this flag
   * set will only be sent if you passed the flag
   * kFSEventStreamCreateFlagWatchRoot to FSEventStreamCreate...() when
   * you created the stream.
   }
	kFSEventStreamEventFlagRootChanged = $00000020;

  {
   * Denotes a special event sent when a volume is mounted underneath
   * one of the paths being monitored. The path in the event is the
   * path to the newly-mounted volume. You will receive one of these
   * notifications for every volume mount event inside the kernel
   * (independent of DiskArbitration). Beware that a newly-mounted
   * volume could contain an arbitrarily large directory hierarchy.
   * Avoid pitfalls like triggering a recursive scan of a non-local
   * filesystem, which you can detect by checking for the absence of
   * the MNT_LOCAL flag in the f_flags returned by statfs(). Also be
   * aware of the MNT_DONTBROWSE flag that is set for volumes which
   * should not be displayed by user interface elements.
   }
	kFSEventStreamEventFlagMount = $00000040;

  {
   * Denotes a special event sent when a volume is unmounted underneath
   * one of the paths being monitored. The path in the event is the
   * path to the directory from which the volume was unmounted. You
   * will receive one of these notifications for every volume unmount
   * event inside the kernel. This is not a substitute for the
   * notifications provided by the DiskArbitration framework; you only
   * get notified after the unmount has occurred. Beware that
   * unmounting a volume could uncover an arbitrarily large directory
   * hierarchy, although Mac OS X never does that.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kFSEventStreamEventFlagUnmount = $00000080; { These flags are only set if you specified the FileEvents}
                                        { flags when creating the stream.}
	kFSEventStreamEventFlagItemCreated = $00000100;
	kFSEventStreamEventFlagItemRemoved = $00000200;
	kFSEventStreamEventFlagItemInodeMetaMod = $00000400;
	kFSEventStreamEventFlagItemRenamed = $00000800;
	kFSEventStreamEventFlagItemModified = $00001000;
	kFSEventStreamEventFlagItemFinderInfoMod = $00002000;
	kFSEventStreamEventFlagItemChangeOwner = $00004000;
	kFSEventStreamEventFlagItemXattrMod = $00008000;
	kFSEventStreamEventFlagItemIsFile = $00010000;
	kFSEventStreamEventFlagItemIsDir = $00020000;
	kFSEventStreamEventFlagItemIsSymlink = $00040000;
	kFSEventStreamEventFlagOwnEvent = $00080000;
=======
	kFSEventStreamEventFlagUnmount = $00000080;
>>>>>>> graemeg/cpstrnew
=======
	kFSEventStreamEventFlagUnmount = $00000080;
>>>>>>> graemeg/cpstrnew
=======
	kFSEventStreamEventFlagUnmount = $00000080;
>>>>>>> graemeg/cpstrnew
=======
	kFSEventStreamEventFlagUnmount = $00000080;
>>>>>>> origin/cpstrnew
=======
	kFSEventStreamEventFlagUnmount = $00000080;
>>>>>>> origin/fixes_2.4


{
 *  FSEventStreamEventId
 *  
 *  Discussion:
 *    Event IDs that can be passed to the FSEventStreamCreate...()
 *    functions and FSEventStreamCallback(). They are monotonically
 *    increasing per system, even across reboots and drives coming and
 *    going. They bear no relation to any particular clock or timebase.
 }
type
	FSEventStreamEventId = UInt64;
	FSEventStreamEventIdPtr = ^FSEventStreamEventId;

const
	kFSEventStreamEventIdSinceNow = $FFFFFFFFFFFFFFFF;


{
 *  FSEventStreamRef
 *  
 *  Discussion:
 *    This is the type of a reference to an FSEventStream.
 }
type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	FSEventStreamRef = ^__FSEventStream; { an opaque type }
	__FSEventStream = record end;
=======
	FSEventStreamRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	FSEventStreamRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	FSEventStreamRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	FSEventStreamRef = ^SInt32; { an opaque type }
>>>>>>> origin/cpstrnew
=======
	FSEventStreamRef = ^SInt32; { an opaque type }
>>>>>>> origin/fixes_2.4

{
 *  ConstFSEventStreamRef
 *  
 *  Discussion:
 *    This is the type of a reference to a constant FSEventStream.
 }
type
	ConstFSEventStreamRef = FSEventStreamRef;

{
 *  FSEventStreamContext
 *  
 *  Discussion:
 *    Structure containing client-supplied data (and callbacks to
 *    manage it) that should be associated with a newly-created stream.
 }
type
	FSEventStreamContextPtr = ^FSEventStreamContext;
	FSEventStreamContext = record
{
   * Currently the only valid value is zero.
   }
		version: CFIndex;

  {
   * An arbitrary client-defined value (for instance, a pointer) to be
   * associated with the stream and passed to the callback when it is
   * invoked.  If a non-NULL value is supplied for the retain callback
   * the framework will use it to retain this value.  If a non-NULL
   * value is supplied for the release callback then when the stream is
   * deallocated it will be used to release this value.  This can be
   * NULL.
   }
		info: UnivPtr;

  {
   * The callback used retain the info pointer.  This can be NULL.
   }
		retain: CFAllocatorRetainCallBack;

  {
   * The callback used release a retain on the info pointer.  This can
   * be NULL.
   }
		release: CFAllocatorReleaseCallBack;

  {
   * The callback used to create a descriptive string representation of
   * the info pointer (or the data pointed to by the info pointer) for
   * debugging purposes.  This can be NULL.
   }
		copyDescription: CFAllocatorCopyDescriptionCallBack;
	end;

{
 *  FSEventStreamCallback
 *  
 *  Discussion:
 *    This is the type of the callback function supplied by the client
 *    when creating a new stream.  This callback is invoked by the
 *    service from the client's runloop(s) when events occur, per the
 *    parameters specified when the stream was created.
 *  
 *  Parameters:
 *    
 *    streamRef:
 *      The stream for which event(s) occurred.
 *    
 *    clientCallBackInfo:
 *      The info field that was supplied in the context when this
 *      stream was created.
 *    
 *    numEvents:
 *      The number of events being reported in this callback. Each of
 *      the arrays (eventPaths, eventFlags, eventIds) will have this
 *      many elements.
 *    
 *    eventPaths:
 *      An array of paths to the directories in which event(s)
 *      occurred. The type of this parameter depends on the flags
 *      passed to FSEventStreamCreate...(). If
 *      kFSEventStreamCreateFlagUseCFTypes was set, then this will be a
 *      CFArrayRef containing CFStringRef objects (per
 *      CFStringCreateWithFileSystemRepresentation()). Ownership
 *      follows the Get rule, and they will be released by the
 *      framework after your callback returns. If
 *      kFSEventStreamCreateFlagUseCFTypes was not set, then the
 *      framework will pass your callback a raw C array of raw C
 *      strings that will be deallocated by the framework after your
 *      callback returns. A path might be "/" if ether of these flags
 *      is set for the event: kFSEventStreamEventFlagUserDropped,
 *      kFSEventStreamEventFlagKernelDropped.
 *    
 *    eventFlags:
 *      An array of flag words corresponding to the paths in the
 *      eventPaths parameter. If no flags are set, then there was some
 *      change in the directory at the specific path supplied in this
 *      event. See FSEventStreamEventFlags.
 *    
 *    eventIds:
 *      An array of FSEventStreamEventIds corresponding to the paths in
 *      the eventPaths parameter. Each event ID comes from the most
 *      recent event being reported in the corresponding directory
 *      named in the eventPaths parameter.  Event IDs all come from a
 *      single global source.  They are guaranteed to always be
 *      increasing, usually in leaps and bounds, even across system
 *      reboots and moving drives from one machine to another.  Just
 *      before invoking your callback your stream is updated so that
 *      calling the accessor FSEventStreamGetLatestEventId() will
 *      return the largest of the values passed in the eventIds
 *      parameter; if you were to stop processing events from this
 *      stream after this callback and resume processing them later
 *      from a newly-created FSEventStream, this is the value you would
 *      pass for the sinceWhen parameter to the
 *      FSEventStreamCreate...() function.
 }
type
	FSEventStreamCallback = procedure( streamRef: ConstFSEventStreamRef; clientCallBackInfo: UnivPtr; numEvents: size_t; eventPaths: UnivPtr; {const} eventFlags: {variable-size-array} FSEventStreamEventFlagsPtr; {const} eventIds: {variable-size-array} FSEventStreamEventIdPtr );
{
 *  Create
 }
{
 *  FSEventStreamCreate()
 *  
 *  Discussion:
 *    Creates a new FS event stream object with the given parameters.
 *    In order to start receiving callbacks you must also call
 *    FSEventStreamScheduleWithRunLoop() and FSEventStreamStart().
 *  
 *  Parameters:
 *    
 *    allocator:
 *      The CFAllocator to be used to allocate memory for the stream. 
 *      Pass NULL or kCFAllocatorDefault to use the current default
 *      allocator.
 *    
 *    callback:
 *      An FSEventStreamCallback which will be called when FS events
 *      occur.
 *    
 *    context:
 *      A pointer to the FSEventStreamContext structure the client
 *      wants to associate with this stream.  Its fields are copied out
 *      into the stream itself so its memory can be released after the
 *      stream is created.  Passing NULL is allowed and has the same
 *      effect as passing a structure whose fields are all set to zero.
 *    
 *    pathsToWatch:
 *      A CFArray of CFStringRefs, each specifying a path to a
 *      directory, signifying the root of a filesystem hierarchy to be
 *      watched for modifications.
 *    
 *    sinceWhen:
 *      The service will supply events that have happened after the
 *      given event ID. To ask for events "since now" pass the constant
 *      kFSEventStreamEventIdSinceNow. Often, clients will supply the
 *      highest-numbered FSEventStreamEventId they have received in a
 *      callback, which they can obtain via the
 *      FSEventStreamGetLatestEventId() accessor. Do not pass zero for
 *      sinceWhen, unless you want to receive events for every
 *      directory modified since "the beginning of time" -- an unlikely
 *      scenario.
 *    
 *    latency:
 *      The number of seconds the service should wait after hearing
 *      about an event from the kernel before passing it along to the
 *      client via its callback. Specifying a larger value may result
 *      in more effective temporal coalescing, resulting in fewer
 *      callbacks and greater overall efficiency.
 *    
 *    flags:
 *      Flags that modify the behavior of the stream being created. See
 *      FSEventStreamCreateFlags.
 *  
 *  Result:
 *    A valid FSEventStreamRef.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function FSEventStreamCreate( allocator: CFAllocatorRef; callback: FSEventStreamCallback; context: FSEventStreamContextPtr { can be NULL }; pathsToWatch: CFArrayRef; sinceWhen: FSEventStreamEventId; latency: CFTimeInterval; flags: FSEventStreamCreateFlags ): FSEventStreamRef; external name '_FSEventStreamCreate';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  FSEventStreamCreateRelativeToDevice()
 *  
 *  Discussion:
 *    Creates a new FS event stream object for a particular device with
 *    the given parameters. In order to start receiving callbacks you
 *    must also call FSEventStreamScheduleWithRunLoop() and
 *    FSEventStreamStart().
 *  
 *  Parameters:
 *    
 *    allocator:
 *      The CFAllocator to be used to allocate memory for the stream. 
 *      Pass NULL or kCFAllocatorDefault to use the current default
 *      allocator.
 *    
 *    callback:
 *      An FSEventStreamCallback which will be called when FS events
 *      occur.
 *    
 *    context:
 *      A pointer to the FSEventStreamContext structure the client
 *      wants to associate with this stream.  Its fields are copied out
 *      into the stream itself so its memory can be released after the
 *      stream is created.
 *    
 *    deviceToWatch:
 *      A dev_t corresponding to the device which you want to receive
 *      notifications from.  The dev_t is the same as the st_dev field
 *      from a stat structure of a file on that device or the f_fsid[0]
 *      field of a statfs structure.  If the value of dev is zero, it
 *      is ignored.
 *    
 *    pathsToWatchRelativeToDevice:
 *      A CFArray of CFStringRefs, each specifying a relative path to a
 *      directory on the device identified by the dev parameter.  The
 *      paths should be relative to the root of the device.  For
 *      example, if a volume "MyData" is mounted at "/Volumes/MyData"
 *      and you want to watch "/Volumes/MyData/Pictures/July", specify
 *      a path string of "Pictures/July".  To watch the root of a
 *      volume pass a path of "" (the empty string).
 *    
 *    sinceWhen:
 *      The service will supply events that have happened after the
 *      given event ID. To ask for events "since now" pass the constant
 *      kFSEventStreamEventIdSinceNow. Often, clients will supply the
 *      highest-numbered FSEventStreamEventId they have received in a
 *      callback, which they can obtain via the
 *      FSEventStreamGetLatestEventId() accessor. Do not pass zero for
 *      sinceWhen, unless you want to receive events for every
 *      directory modified since "the beginning of time" -- an unlikely
 *      scenario.
 *    
 *    latency:
 *      The number of seconds the service should wait after hearing
 *      about an event from the kernel before passing it along to the
 *      client via its callback. Specifying a larger value may result
 *      in more effective temporal coalescing, resulting in fewer
 *      callbacks.
 *    
 *    flags:
 *      Flags that modify the behavior of the stream being created. See
 *      FSEventStreamCreateFlags.
 *  
 *  Result:
 *    A valid FSEventStreamRef.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function FSEventStreamCreateRelativeToDevice( allocator: CFAllocatorRef; callback: FSEventStreamCallback; var context: FSEventStreamContext; deviceToWatch: dev_t; pathsToWatchRelativeToDevice: CFArrayRef; sinceWhen: FSEventStreamEventId; latency: CFTimeInterval; flags: FSEventStreamCreateFlags ): FSEventStreamRef; external name '_FSEventStreamCreateRelativeToDevice';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  Accessors
 }
{
 *  FSEventStreamGetLatestEventId()
 *  
 *  Discussion:
 *    Fetches the sinceWhen property of the stream.  Upon receiving an
 *    event (and just before invoking the client's callback) this
 *    attribute is updated to the highest-numbered event ID mentioned
 *    in the event.
 *  
 *  Parameters:
 *    
 *    streamRef:
 *      A valid stream.
 *  
 *  Result:
 *    The sinceWhen attribute of the stream.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function FSEventStreamGetLatestEventId( streamRef: ConstFSEventStreamRef ): FSEventStreamEventId; external name '_FSEventStreamGetLatestEventId';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  FSEventStreamGetDeviceBeingWatched()
 *  
 *  Discussion:
 *    Fetches the dev_t supplied when the stream was created via
 *    FSEventStreamCreateRelativeToDevice(), otherwise 0.
 *  
 *  Parameters:
 *    
 *    streamRef:
 *      A valid stream.
 *  
 *  Result:
 *    The dev_t for a device-relative stream, otherwise 0.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function FSEventStreamGetDeviceBeingWatched( streamRef: ConstFSEventStreamRef ): dev_t; external name '_FSEventStreamGetDeviceBeingWatched';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  FSEventStreamCopyPathsBeingWatched()
 *  
 *  Discussion:
 *    Fetches the paths supplied when the stream was created via one of
 *    the FSEventStreamCreate...() functions.
 *  
 *  Parameters:
 *    
 *    streamRef:
 *      A valid stream.
 *  
 *  Result:
 *    A CFArray of CFStringRefs corresponding to those supplied when
 *    the stream was created. Ownership follows the Copy rule.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function FSEventStreamCopyPathsBeingWatched( streamRef: ConstFSEventStreamRef ): CFArrayRef; external name '_FSEventStreamCopyPathsBeingWatched';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  FSEventsGetCurrentEventId()
 *  
 *  Discussion:
 *    Fetches the most recently generated event ID, system-wide (not
 *    just for one stream). By thetime it is returned to your
 *    application even newer events may have already been generated.
 *  
 *  Result:
 *    The event ID of the most recent event generated by the system.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function FSEventsGetCurrentEventId: FSEventStreamEventId; external name '_FSEventsGetCurrentEventId';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  FSEventsCopyUUIDForDevice()
 *  
 *  Discussion:
 *    Gets the UUID associated with a device, or NULL if not possible
 *    (for example, on read-only device).  A (non-NULL) UUID uniquely
 *    identifies a given stream of FSEvents.  If this (non-NULL) UUID
 *    is different than one that you stored from a previous run then
 *    the event stream is different (for example, because FSEvents were
 *    purged, because the disk was erased, or because the event ID
 *    counter wrapped around back to zero). A NULL return value
 *    indicates that "historical" events are not available, i.e., you
 *    should not supply a "sinceWhen" value to FSEventStreamCreate...()
 *    other than kFSEventStreamEventIdSinceNow.
 *  
 *  Parameters:
 *    
 *    dev:
 *      The dev_t of the device that you want to get the UUID for.
 *  
 *  Result:
 *    The UUID associated with the stream of events on this device, or
 *    NULL if no UUID is available (for example, on a read-only
 *    device).  The UUID is stored on the device itself and travels
 *    with it even when the device is attached to different computers. 
 *    Ownership follows the Copy Rule.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function FSEventsCopyUUIDForDevice( dev: dev_t ): CFUUIDRef; external name '_FSEventsCopyUUIDForDevice';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  FSEventsGetLastEventIdForDeviceBeforeTime()
 *  
 *  Discussion:
 *    Gets the last event ID for the given device that was returned
 *    before the given time.  This is conservative in the sense that if
 *    you then use the returned event ID as the sinceWhen parameter of
 *    FSEventStreamCreateRelativeToDevice() that you will not miss any
 *    events that happened since that time.  On the other hand, you
 *    might receive some (harmless) extra events. Beware: there are
 *    things that can cause this to fail to be accurate. For example,
 *    someone might change the system's clock (either backwards or
 *    forwards).  Or an external drive might be used on different
 *    systems without perfectly synchronized clocks.
 *  
 *  Parameters:
 *    
 *    dev:
 *      The dev_t of the device.
 *    
 *    time:
 *      The time as a CFAbsoluteTime whose value is the number of
 *      seconds since Jan 1, 1970 (i.e. a posix style time_t).
 *  
 *  Result:
 *    The last event ID for the given device that was returned before
 *    the given time.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function FSEventsGetLastEventIdForDeviceBeforeTime( dev: dev_t; time: CFAbsoluteTime ): FSEventStreamEventId; external name '_FSEventsGetLastEventIdForDeviceBeforeTime';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  FSEventsPurgeEventsForDeviceUpToEventId()
 *  
 *  Discussion:
 *    Purges old events from the persistent per-volume database
 *    maintained by the service. Can only be called by the root user.
 *  
 *  Parameters:
 *    
 *    dev:
 *      The dev_t of the device.
 *    
 *    eventId:
 *      The event ID.
 *  
 *  Result:
 *    True if it succeeds, otherwise False if it fails.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function FSEventsPurgeEventsForDeviceUpToEventId( dev: dev_t; eventId: FSEventStreamEventId ): Boolean; external name '_FSEventsPurgeEventsForDeviceUpToEventId';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  Retain, Release
 }
{
 *    @function FSEventStreamRetain
 *    Increments the stream's refcount.  The refcount is initially one and is
 *    decremented via FSEventStreamRelease().
 *  
 *    @param streamRef
 *      A valid stream.
 *    
 }
{
 *  FSEventStreamRetain()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
procedure FSEventStreamRetain( streamRef: FSEventStreamRef ); external name '_FSEventStreamRetain';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  FSEventStreamRelease()
 *  
 *  Discussion:
 *    Decrements the stream's refcount.  The refcount is initially one
 *    and is incremented via FSEventStreamRetain().  If the refcount
 *    reaches zero then the stream is deallocated.
 *  
 *  Parameters:
 *    
 *    streamRef:
 *      A valid stream.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
procedure FSEventStreamRelease( streamRef: FSEventStreamRef ); external name '_FSEventStreamRelease';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  ScheduleWithRunLoop, UnscheduleFromRunLoop, Invalidate
 }
{
 *  FSEventStreamScheduleWithRunLoop()
 *  
 *  Discussion:
 *    This function schedules the stream on the specified run loop,
 *    like CFRunLoopAddSource() does for a CFRunLoopSourceRef.  The
 *    caller is responsible for ensuring that the stream is scheduled
 *    on at least one run loop and that at least one of the run loops
 *    on which the stream is scheduled is being run. To start receiving
 *    events on the stream, call FSEventStreamStart(). To remove the
 *    stream from the run loops upon which it has been scheduled, call
 *    FSEventStreamUnscheduleFromRunLoop() or FSEventStreamInvalidate().
 *  
 *  Parameters:
 *    
 *    streamRef:
 *      A valid stream.
 *    
 *    runLoop:
 *      The run loop on which to schedule the stream.
 *    
 *    runLoopMode:
 *      A run loop mode on which to schedule the stream.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
procedure FSEventStreamScheduleWithRunLoop( streamRef: FSEventStreamRef; runLoop: CFRunLoopRef; runLoopMode: CFStringRef ); external name '_FSEventStreamScheduleWithRunLoop';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  FSEventStreamUnscheduleFromRunLoop()
 *  
 *  Discussion:
 *    This function removes the stream from the specified run loop,
 *    like CFRunLoopRemoveSource() does for a CFRunLoopSourceRef.
 *  
 *  Parameters:
 *    
 *    streamRef:
 *      A valid stream.
 *    
 *    runLoop:
 *      The run loop from which to unschedule the stream.
 *    
 *    runLoopMode:
 *      The run loop mode from which to unschedule the stream.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
procedure FSEventStreamUnscheduleFromRunLoop( streamRef: FSEventStreamRef; runLoop: CFRunLoopRef; runLoopMode: CFStringRef ); external name '_FSEventStreamUnscheduleFromRunLoop';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  FSEventStreamSetDispatchQueue()
 *  
 *  Discussion:
 *    This function schedules the stream on the specified dispatch
 *    queue. The caller is responsible for ensuring that the stream is
 *    scheduled on a dispatch queue and that the queue is started. If
 *    there is a problem scheduling the stream on the queue an error
 *    will be returned when you try to Start the stream. To start
 *    receiving events on the stream, call FSEventStreamStart(). To
 *    remove the stream from the queue on which it was scheduled, call
 *    FSEventStreamSetDispatchQueue() with a NULL queue parameter or
 *    call FSEventStreamInvalidate() which will do the same thing. 
 *    Note: you must eventually call FSEventStreamInvalidate() and it
 *    is an error to call FSEventStreamInvalidate() without having the
 *    stream either scheduled on a runloop or a dispatch queue, so do
 *    not set the dispatch queue to NULL before calling
 *    FSEventStreamInvalidate().
 *  
 *  Parameters:
 *    
 *    streamRef:
 *      A valid stream.
 *    
 *    q:
 *      The dispatch queue to use to receive events (or NULL to to stop
 *      receiving events from the stream).
 *  
 *  Availability:
 *    Mac OS X:         in version 10.6 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
(*
Requires translation of libdispatch headers
procedure FSEventStreamSetDispatchQueue( streamRef: FSEventStreamRef; q: dispatch_queue_t ); external name '_FSEventStreamSetDispatchQueue';
*)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
// __OSX_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA);
=======
// AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER;
>>>>>>> graemeg/cpstrnew
=======
// AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER;
>>>>>>> graemeg/cpstrnew
=======
// AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER;
>>>>>>> graemeg/cpstrnew
=======
// AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER;
>>>>>>> origin/cpstrnew
=======
// AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER;
>>>>>>> origin/fixes_2.4


{
 *  FSEventStreamInvalidate()
 *  
 *  Discussion:
 *    Invalidates the stream, like CFRunLoopSourceInvalidate() does for
 *    a CFRunLoopSourceRef.  It will be unscheduled from any runloops
 *    or dispatch queues upon which it had been scheduled.
 *    FSEventStreamInvalidate() can only be called on the stream after
 *    you have called FSEventStreamScheduleWithRunLoop() or
 *    FSEventStreamSetDispatchQueue().
 *  
 *  Parameters:
 *    
 *    streamRef:
 *      A valid stream.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
procedure FSEventStreamInvalidate( streamRef: FSEventStreamRef ); external name '_FSEventStreamInvalidate';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  Start, Flush, Stop
 }
{
 *  FSEventStreamStart()
 *  
 *  Discussion:
 *    Attempts to register with the FS Events service to receive events
 *    per the parameters in the stream. FSEventStreamStart() can only
 *    be called once the stream has been scheduled on at least one
 *    runloop, via FSEventStreamScheduleWithRunLoop(). Once started,
 *    the stream can be stopped via FSEventStreamStop().
 *  
 *  Parameters:
 *    
 *    streamRef:
 *      A valid stream.
 *  
 *  Result:
 *    True if it succeeds, otherwise False if it fails.  It ought to
 *    always succeed, but in the event it does not then your code
 *    should fall back to performing recursive scans of the directories
 *    of interest as appropriate.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function FSEventStreamStart( streamRef: FSEventStreamRef ): Boolean; external name '_FSEventStreamStart';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  FSEventStreamFlushAsync()
 *  
 *  Discussion:
 *    Asks the FS Events service to flush out any events that have
 *    occurred but have not yet been delivered, due to the latency
 *    parameter that was supplied when the stream was created.  This
 *    flushing occurs asynchronously -- do not expect the events to
 *    have already been delivered by the time this call returns.
 *    FSEventStreamFlushAsync() can only be called after the stream has
 *    been started, via FSEventStreamStart().
 *  
 *  Parameters:
 *    
 *    streamRef:
 *      A valid stream.
 *  
 *  Result:
 *    The largest event id of any event ever queued for this stream,
 *    otherwise zero if no events have been queued for this stream.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function FSEventStreamFlushAsync( streamRef: FSEventStreamRef ): FSEventStreamEventId; external name '_FSEventStreamFlushAsync';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  FSEventStreamFlushSync()
 *  
 *  Discussion:
 *    Asks the FS Events service to flush out any events that have
 *    occurred but have not yet been delivered, due to the latency
 *    parameter that was supplied when the stream was created.  This
 *    flushing occurs synchronously -- by the time this call returns,
 *    your callback will have been invoked for every event that had
 *    already occurred at the time you made this call.
 *    FSEventStreamFlushSync() can only be called after the stream has
 *    been started, via FSEventStreamStart().
 *  
 *  Parameters:
 *    
 *    streamRef:
 *      A valid stream.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
procedure FSEventStreamFlushSync( streamRef: FSEventStreamRef ); external name '_FSEventStreamFlushSync';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  FSEventStreamStop()
 *  
 *  Discussion:
 *    Unregisters with the FS Events service.  The client callback will
 *    not be called for this stream while it is stopped.
 *    FSEventStreamStop() can only be called if the stream has been
 *    started, via FSEventStreamStart(). Once stopped, the stream can
 *    be restarted via FSEventStreamStart(), at which point it will
 *    resume receiving events from where it left off ("sinceWhen").
 *  
 *  Parameters:
 *    
 *    streamRef:
 *      A valid stream.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
procedure FSEventStreamStop( streamRef: FSEventStreamRef ); external name '_FSEventStreamStop';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  Debugging
 }
{
 *  FSEventStreamShow()
 *  
 *  Discussion:
 *    Prints a description of the supplied stream to stderr. For
 *    debugging only.
 *  
 *  Parameters:
 *    
 *    streamRef:
 *      A valid stream.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
procedure FSEventStreamShow( streamRef: ConstFSEventStreamRef ); external name '_FSEventStreamShow';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  FSEventStreamCopyDescription()
 *  
 *  Discussion:
 *    Returns a CFStringRef containing the description of the supplied
 *    stream. For debugging only.
 *  
 *  Result:
 *    A CFStringRef containing the description of the supplied stream.
 *    Ownership follows the Copy rule.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function FSEventStreamCopyDescription( streamRef: ConstFSEventStreamRef ): CFStringRef; external name '_FSEventStreamCopyDescription';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_NA) *)


{
 * FSEventStreamSetExclusionPaths()
 *
 * Discussion: 
 *    Sets directories to be filtered from the EventStream.
 *    A maximum of 8 directories maybe specified.
 *
 * Result:
 *    True if it succeeds, otherwise False if it fails.
 *     
 * Availability:
 *    Mac OS X:         in version 10.9 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }

function FSEventStreamSetExclusionPaths( streamRef: FSEventStreamRef; pathsToExclude: CFArrayRef ): Boolean; external name '_FSEventStreamSetExclusionPaths';
(* __OSX_AVAILABLE_STARTING(__MAC_10_9, __IPHONE_NA) *)


{$endc} {TARGET_OS_MAC}

=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{$endc} {TARGET_OS_MAC}
<<<<<<< HEAD
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
=======
>>>>>>> origin/fixes_2.4
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
