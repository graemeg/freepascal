{
 * Copyright (c) 2003 Apple Computer, Inc. All Rights Reserved.
 * 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 * @APPLE_LICENSE_HEADER_START@
 * 
 * This file contains Original Code and/or Modifications of Original Code
 * as defined in and that are subject to the Apple Public Source License
 * Version 2.0 (the 'License'). You may not use this file except in
 * compliance with the License. Please obtain a copy of the License at
 * http://www.opensource.apple.com/apsl/ and read it before using this
 * file.
 * 
 * The Original Code and all software distributed under the License are
 * distributed on an 'AS IS' basis, WITHOUT WARRANTY OF ANY KIND, EITHER
 * EXPRESS OR IMPLIED, AND APPLE HEREBY DISCLAIMS ALL SUCH WARRANTIES,
 * INCLUDING WITHOUT LIMITATION, ANY WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE, QUIET ENJOYMENT OR NON-INFRINGEMENT.
 * Please see the License for the specific language governing rights and
 * limitations under the License.
 * 
 * @APPLE_LICENSE_HEADER_END@
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{  Pascal Translation:  Gorazd Krosl <gorazd_1957@yahoo.ca>, October 2009 }
{  Pascal Translation Update: Jonas Maebe <jonas@freepascal.org>, October 2012 }
{  Pascal Translation Update: Jonas Maebe <jonas@freepascal.org>, August 2015 }
=======
{	  Pascal Translation:  Gorazd Krosl <gorazd_1957@yahoo.ca>, October 2009 }
>>>>>>> graemeg/cpstrnew
=======
{	  Pascal Translation:  Gorazd Krosl <gorazd_1957@yahoo.ca>, October 2009 }
>>>>>>> graemeg/cpstrnew
=======
{	  Pascal Translation:  Gorazd Krosl <gorazd_1957@yahoo.ca>, October 2009 }
>>>>>>> graemeg/cpstrnew
=======
{	  Pascal Translation:  Gorazd Krosl <gorazd_1957@yahoo.ca>, October 2009 }
>>>>>>> origin/cpstrnew

{
    Modified for use with Free Pascal
    Version 308
    Please report any bugs to <gpc@microbizz.nl>
}

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
 * The contents of this file constitute Original Code as defined in and are
 * subject to the Apple Public Source License Version 1.2 (the 'License').
 * You may not use this file except in compliance with the License. Please obtain
 * a copy of the License at http://www.apple.com/publicsource and read it before
 * using this file.
=======
 * @APPLE_LICENSE_HEADER_START@
>>>>>>> origin/fixes_2.4
 * 
 * This file contains Original Code and/or Modifications of Original Code
 * as defined in and that are subject to the Apple Public Source License
 * Version 2.0 (the 'License'). You may not use this file except in
 * compliance with the License. Please obtain a copy of the License at
 * http://www.opensource.apple.com/apsl/ and read it before using this
 * file.
 * 
 * The Original Code and all software distributed under the License are
 * distributed on an 'AS IS' basis, WITHOUT WARRANTY OF ANY KIND, EITHER
 * EXPRESS OR IMPLIED, AND APPLE HEREBY DISCLAIMS ALL SUCH WARRANTIES,
 * INCLUDING WITHOUT LIMITATION, ANY WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE, QUIET ENJOYMENT OR NON-INFRINGEMENT.
 * Please see the License for the specific language governing rights and
 * limitations under the License.
 * 
 * @APPLE_LICENSE_HEADER_END@
 }
{	  Pascal Translation:  Gorazd Krosl <gorazd_1957@yahoo.ca>, October 2009 }

{
    Modified for use with Free Pascal
    Version 308
    Please report any bugs to <gpc@microbizz.nl>
}

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}
>>>>>>> origin/fixes_2.4
{$mode macpas}
{$packenum 1}
{$macro on}
{$inline on}
{$calling mwpascal}

unit AuthorizationDB;
interface
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$setc UNIVERSAL_INTERFACES_VERSION := $0400}
{$setc GAP_INTERFACES_VERSION := $0308}
=======
{$setc UNIVERSAL_INTERFACES_VERSION := $0342}
{$setc GAP_INTERFACES_VERSION := $0210}
>>>>>>> graemeg/fixes_2_2
=======
{$setc UNIVERSAL_INTERFACES_VERSION := $0342}
{$setc GAP_INTERFACES_VERSION := $0210}
>>>>>>> origin/fixes_2_2
=======
{$setc UNIVERSAL_INTERFACES_VERSION := $0400}
{$setc GAP_INTERFACES_VERSION := $0308}
>>>>>>> origin/fixes_2.4

{$ifc not defined USE_CFSTR_CONSTANT_MACROS}
    {$setc USE_CFSTR_CONSTANT_MACROS := TRUE}
{$endc}

{$ifc defined CPUPOWERPC and defined CPUI386}
	{$error Conflicting initial definitions for CPUPOWERPC and CPUI386}
{$endc}
{$ifc defined FPC_BIG_ENDIAN and defined FPC_LITTLE_ENDIAN}
	{$error Conflicting initial definitions for FPC_BIG_ENDIAN and FPC_LITTLE_ENDIAN}
{$endc}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifc not defined __ppc__ and defined CPUPOWERPC32}
=======
{$ifc not defined __ppc__ and defined CPUPOWERPC}
>>>>>>> graemeg/fixes_2_2
=======
{$ifc not defined __ppc__ and defined CPUPOWERPC}
>>>>>>> origin/fixes_2_2
=======
{$ifc not defined __ppc__ and defined CPUPOWERPC32}
>>>>>>> origin/fixes_2.4
	{$setc __ppc__ := 1}
{$elsec}
	{$setc __ppc__ := 0}
{$endc}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
{$ifc not defined __ppc64__ and defined CPUPOWERPC64}
	{$setc __ppc64__ := 1}
{$elsec}
	{$setc __ppc64__ := 0}
{$endc}
<<<<<<< HEAD
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
{$ifc not defined __i386__ and defined CPUI386}
	{$setc __i386__ := 1}
{$elsec}
	{$setc __i386__ := 0}
{$endc}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
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

<<<<<<< HEAD
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4

{$ifc defined __ppc__ and __ppc__ and defined __i386__ and __i386__}
	{$error Conflicting definitions for __ppc__ and __i386__}
{$endc}

{$ifc defined __ppc__ and __ppc__}
	{$setc TARGET_CPU_PPC := TRUE}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	{$setc TARGET_CPU_PPC64 := FALSE}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_CPU_ARM64 := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
	{$setc TARGET_OS_EMBEDDED := FALSE}
{$elifc defined __ppc64__ and __ppc64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := TRUE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_CPU_ARM64 := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
	{$setc TARGET_OS_EMBEDDED := FALSE}
=======
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$elifc defined __ppc64__ and __ppc64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := TRUE}
=======
>>>>>>> graemeg/cpstrnew
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
{$elifc defined __ppc64__ and __ppc64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := TRUE}
=======
>>>>>>> graemeg/cpstrnew
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
{$elifc defined __ppc64__ and __ppc64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := TRUE}
=======
>>>>>>> origin/cpstrnew
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
{$elifc defined __ppc64__ and __ppc64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := TRUE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> origin/cpstrnew
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
	{$setc TARGET_CPU_ARM64 := FALSE}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{$ifc defined(iphonesim)}
 	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := TRUE}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/cpstrnew
{$elsec}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$endc}
<<<<<<< HEAD
<<<<<<< HEAD
	{$setc TARGET_OS_EMBEDDED := FALSE}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{$elifc defined __x86_64__ and __x86_64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := TRUE}
	{$setc TARGET_CPU_ARM := FALSE}
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
{$elifc defined __arm__ and __arm__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := TRUE}
<<<<<<< HEAD
	{$setc TARGET_CPU_ARM64 := FALSE}
=======
>>>>>>> graemeg/cpstrnew
	{ will require compiler define when/if other Apple devices with ARM cpus ship }
	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
<<<<<<< HEAD
	{$setc TARGET_OS_EMBEDDED := TRUE}
{$elifc defined __arm64__ and __arm64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_CPU_ARM64 := TRUE}
	{ will require compiler define when/if other Apple devices with ARM cpus ship }
	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
	{$setc TARGET_OS_EMBEDDED := TRUE}
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ nor __arm64__ is defined.}
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
=======
{$elsec}
=======
=======
>>>>>>> graemeg/cpstrnew
{$elsec}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$endc}
{$elifc defined __x86_64__ and __x86_64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := TRUE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$elifc defined __arm__ and __arm__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := TRUE}
	{ will require compiler define when/if other Apple devices with ARM cpus ship }
	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$elsec}
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$elifc defined __arm__ and __arm__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := TRUE}
	{ will require compiler define when/if other Apple devices with ARM cpus ship }
	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$elsec}
>>>>>>> origin/cpstrnew
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
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
  {$setc TARGET_CPU_64 := FALSE}
{$endc}
=======
=======
>>>>>>> origin/fixes_2_2
=======
	{$setc TARGET_CPU_PPC64 := FALSE}
>>>>>>> origin/fixes_2.4
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$elifc defined __ppc64__ and __ppc64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := TRUE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$elifc defined __i386__ and __i386__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := TRUE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
{$ifc defined(iphonesim)}
 	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := TRUE}
{$elsec}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$endc}
{$elifc defined __x86_64__ and __x86_64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := TRUE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$elifc defined __arm__ and __arm__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := TRUE}
	{ will require compiler define when/if other Apple devices with ARM cpus ship }
	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
  {$setc TARGET_CPU_64 := FALSE}
{$endc}
<<<<<<< HEAD
{$setc TARGET_CPU_PPC_64 := FALSE}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4

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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
{$setc TARGET_OS_MAC := TRUE}
>>>>>>> graemeg/fixes_2_2
=======
{$setc TARGET_OS_MAC := TRUE}
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
{$setc TARGET_OS_UNIX := FALSE}
{$setc TARGET_OS_WIN32 := FALSE}
{$setc TARGET_RT_MAC_68881 := FALSE}
{$setc TARGET_RT_MAC_CFM := FALSE}
{$setc TARGET_RT_MAC_MACHO := TRUE}
{$setc TYPED_FUNCTION_POINTERS := TRUE}
{$setc TYPE_BOOL := FALSE}
{$setc TYPE_EXTENDED := FALSE}
{$setc TYPE_LONGLONG := TRUE}
uses MacTypes,Authorization,CFBase,CFDictionary,CFString,CFBundle;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
{$endc} {not MACOSALLINCLUDE}


{$ifc TARGET_OS_MAC}


{$ALIGN POWER}


{
 *  AuthorizationDB.h -- APIs for managing the authorization policy database
 *  and daemons.
 }


<<<<<<< HEAD
=======
{$ALIGN MAC68K}
>>>>>>> graemeg/fixes_2_2
=======
{$ALIGN MAC68K}
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
{!
	@header AuthorizationDB
	Version 1.0

	This API allows for any programs to get, modify, delete and add new right definitions to the policy database.  Meta-rights specify whether and what authorization is required to make these modifications.
	
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	AuthorizationRightSet(authRef, "com.ifoo.ifax.send", CFSTR(kRuleIsAdmin), CFSTR("You must authenticate to send a fax."), NULL, NULL)

	add a rule for letting admins send faxes using a canned rule, delegating to a pre-specified rule that authorizes everyone who is an admin.
	
	AuthorizationRightSet(authRef, "com.ifoo.ifax.send", [[CFSTR(kRightRule), CFSTR(kRuleIsAdmin)], [CFSTR(kRightComment), CFSTR("authorizes sending of 1 fax message")]], CFSTR("Authorize sending of a fax"), NULL, NULL)

	add identical rule, but specify additional attributes this time.

	Keep in mind while specifying a comment to be specific about what you need to authorize for (1 fax), in terms of a general message for user.  The means of proof required for kRuleIsAdmin (enter username/password for example) should not be included here, since it could be configured differently.  Also note that the "authRef" variable used in each of the above examples must be a vaild AuthorizationRef obtained from AuthorizationCreate().
=======
=======
>>>>>>> origin/fixes_2_2
	AuthorizationRightSet(NULL, "com.ifoo.ifax.send", CFSTR(kRuleIsAdmin), CFSTR("You must authenticate to send a fax."), NULL, NULL)
=======
	AuthorizationRightSet(authRef, "com.ifoo.ifax.send", CFSTR(kRuleIsAdmin), CFSTR("You must authenticate to send a fax."), NULL, NULL)
>>>>>>> origin/fixes_2.4

	add a rule for letting admins send faxes using a canned rule, delegating to a pre-specified rule that authorizes everyone who is an admin.
	
	AuthorizationRightSet(authRef, "com.ifoo.ifax.send", [[CFSTR(kRightRule), CFSTR(kRuleIsAdmin)], [CFSTR(kRightComment), CFSTR("authorizes sending of 1 fax message")]], CFSTR("Authorize sending of a fax"), NULL, NULL)

	add identical rule, but specify additional attributes this time.

<<<<<<< HEAD
	Keep in mind while specifying a comment to be specific about what you need to authorize for (1 fax), in terms of a general message for user.  The means of proof required for kRuleIsAdmin (enter username/password for example) should not be included here, since it could be configured differently.
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	Keep in mind while specifying a comment to be specific about what you need to authorize for (1 fax), in terms of a general message for user.  The means of proof required for kRuleIsAdmin (enter username/password for example) should not be included here, since it could be configured differently.  Also note that the "authRef" variable used in each of the above examples must be a vaild AuthorizationRef obtained from AuthorizationCreate().
>>>>>>> origin/fixes_2.4

}

{!	@define kRightRule
	rule delegation key.  Instead of specifying exact behavior some canned rules
   are shipped that may be switched by configurable security.
}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
const
	kAuthorizationRightRule = 'rule';
=======
const kAuthorizationRightRule					= 'rule';
>>>>>>> graemeg/fixes_2_2
=======
const kAuthorizationRightRule					= 'rule';
>>>>>>> origin/fixes_2_2
=======
const
	kAuthorizationRightRule = 'rule';
>>>>>>> origin/fixes_2.4

{! @defined kRuleIsAdmin
	canned rule values for use with rule delegation definitions: require user to be an admin.
}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
const
	kAuthorizationRuleIsAdmin = 'is-admin';
=======
const kAuthorizationRuleIsAdmin				= 'is-admin';
>>>>>>> graemeg/fixes_2_2
=======
const kAuthorizationRuleIsAdmin				= 'is-admin';
>>>>>>> origin/fixes_2_2
=======
const
	kAuthorizationRuleIsAdmin = 'is-admin';
>>>>>>> origin/fixes_2.4

{! @defined kRuleAuthenticateAsSessionUser
	canned rule value for use with rule delegation definitions: require user to authenticate as the session owner (logged-in user).
}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
const
	kAuthorizationRuleAuthenticateAsSessionUser = 'authenticate-session-owner';
=======
const kAuthorizationRuleAuthenticateAsSessionUser= 'authenticate-session-user';
>>>>>>> graemeg/fixes_2_2
=======
const kAuthorizationRuleAuthenticateAsSessionUser= 'authenticate-session-user';
>>>>>>> origin/fixes_2_2
=======
const
	kAuthorizationRuleAuthenticateAsSessionUser = 'authenticate-session-owner';
>>>>>>> origin/fixes_2.4

{! @defined kRuleAuthenticateAsAdmin
	Canned rule value for use with rule delegation definitions: require user to authenticate as admin.
}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
const
	kAuthorizationRuleAuthenticateAsAdmin = 'authenticate-admin';
=======
const kAuthorizationRuleAuthenticateAsAdmin	= 'authenticate-admin';
>>>>>>> graemeg/fixes_2_2
=======
const kAuthorizationRuleAuthenticateAsAdmin	= 'authenticate-admin';
>>>>>>> origin/fixes_2_2
=======
const
	kAuthorizationRuleAuthenticateAsAdmin = 'authenticate-admin';
>>>>>>> origin/fixes_2.4

{! @defined kAuthorizationRuleClassAllow
	Class that allows anything.
}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
const
	kAuthorizationRuleClassAllow = 'allow';
=======
const kAuthorizationRuleClassAllow		= 'allow';
>>>>>>> graemeg/fixes_2_2
=======
const kAuthorizationRuleClassAllow		= 'allow';
>>>>>>> origin/fixes_2_2
=======
const
	kAuthorizationRuleClassAllow = 'allow';
>>>>>>> origin/fixes_2.4

{! @defined kAuthorizationRuleClassDeny
	Class that denies anything. 
}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
const
	kAuthorizationRuleClassDeny = 'deny';
=======
const kAuthorizationRuleClassDeny			= 'deny';
>>>>>>> graemeg/fixes_2_2
=======
const kAuthorizationRuleClassDeny			= 'deny';
>>>>>>> origin/fixes_2_2
=======
const
	kAuthorizationRuleClassDeny = 'deny';
>>>>>>> origin/fixes_2.4

{! @defined kAuthorizationComment
    comments for the administrator on what is being customized here;
   as opposed to (localized) descriptions presented to the user.
}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
const
	kAuthorizationComment = 'comment';
=======
const kAuthorizationComment = 'comment';
>>>>>>> graemeg/fixes_2_2
=======
const kAuthorizationComment = 'comment';
>>>>>>> origin/fixes_2_2
=======
const
	kAuthorizationComment = 'comment';
>>>>>>> origin/fixes_2.4


{!
	@function AuthorizationRightGet 
	
	Retrieves a right definition as a dictionary.  There are no restrictions to keep anyone from retrieving these definitions.  

	@param rightName (input) the rightname (ASCII).  Wildcard rightname definitions are okay.
	@param rightDefinition (output/optional) the dictionary with all keys defining the right.  See documented keys.  Passing in NULL will just check if there is a definition.  The caller is responsible for releasing the returned dictionary.

	@result errAuthorizationSuccess 0 No error.

	errAuthorizationDenied -60005 No definition found.

}
function AuthorizationRightGet( rightName: ConstCStringPtr; rightDefinition: CFDictionaryRefPtr ): OSStatus; external name '_AuthorizationRightGet';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
// attribute(weak_import);
>>>>>>> graemeg/fixes_2_2
=======
// attribute(weak_import);
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4

{!
	@function AuthorizationRightSet
	
	Create or update a right entry.  Only normal rights can be registered (wildcard rights are denied); wildcard rights are considered to be put in by an administrator putting together a site configuration.

	@param authRef (input) authRef to authorize modifications.
	@param rightName (input) the rightname (ASCII).  Wildcard rightnames are not okay.
	@param rightDefinition (input) a CFString of the name of a rule to use (delegate) or CFDictionary containing keys defining one.
	@param descriptionKey (input/optional) a CFString to use as a key for looking up localized descriptions.  If no localization is found this will be the description itself.
	@param bundle (input/optional) a bundle to get localizations from if not the main bundle.
	@param localeTableName (input/optional) stringtable name to get localizations from.
	
	@result errAuthorizationSuccess 0 added right definition successfully.

	errAuthorizationDenied -60005 Unable to create or update right definition.

	errAuthorizationCanceled -60006 Authorization was canceled by user.

	errAuthorizationInteractionNotAllowed -60007 Interaction was required but not possible.

}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function AuthorizationRightSet( authRef: AuthorizationRef; rightName: ConstCStringPtr; rightDefinition: CFTypeRef; descriptionKey: CFStringRef; bundle: CFBundleRef; localeTableName: CFStringRef ): OSStatus; external name '_AuthorizationRightSet';
=======
function AuthorizationRightSet(authRef: AuthorizationRef; rightName: ConstCStringPtr; rightDefinition: CFTypeRef; descriptionKey: CFStringRef; bundle: CFBundleRef; localeTableName: CFStringRef): OSStatus; external name '_AuthorizationRightSet';
// attribute(weak_import);
>>>>>>> graemeg/fixes_2_2
=======
function AuthorizationRightSet(authRef: AuthorizationRef; rightName: ConstCStringPtr; rightDefinition: CFTypeRef; descriptionKey: CFStringRef; bundle: CFBundleRef; localeTableName: CFStringRef): OSStatus; external name '_AuthorizationRightSet';
// attribute(weak_import);
>>>>>>> origin/fixes_2_2
=======
function AuthorizationRightSet( authRef: AuthorizationRef; rightName: ConstCStringPtr; rightDefinition: CFTypeRef; descriptionKey: CFStringRef; bundle: CFBundleRef; localeTableName: CFStringRef ): OSStatus; external name '_AuthorizationRightSet';
>>>>>>> origin/fixes_2.4


{!
	@function AuthorizationRightRemove

	Request to remove a right from the policy database.

	@param authRef (input) authRef, to be used to authorize this action.
	@param rightName (input) the rightname (ASCII).  Wildcard rightnames are not okay.
	
}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
function AuthorizationRightRemove( authRef: AuthorizationRef; rightName: ConstCStringPtr ): OSStatus; external name '_AuthorizationRightRemove';

{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}
<<<<<<< HEAD

end.
{$endc} {not MACOSALLINCLUDE}
=======
function AuthorizationRightRemove(authRef: AuthorizationRef; rightName: ConstCStringPtr): OSStatus; external name '_AuthorizationRightRemove';

end.
>>>>>>> graemeg/fixes_2_2
=======
function AuthorizationRightRemove(authRef: AuthorizationRef; rightName: ConstCStringPtr): OSStatus; external name '_AuthorizationRightRemove';

end.
>>>>>>> origin/fixes_2_2
=======

end.
{$endc} {not MACOSALLINCLUDE}
>>>>>>> origin/fixes_2.4
