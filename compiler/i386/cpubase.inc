{
    Copyright (c) 1998-2000 by Florian Klaempfl and Peter Vreman

    Contains the basic declarations for the i386 architecture

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.

 ****************************************************************************
}
{ This include file contains the basic declarations for the i386 architecture.
}

{*****************************************************************************
                                Operand Sizes
*****************************************************************************}

    type
      topsize = (S_NO,
        S_B,S_W,S_L,S_Q,S_BW,S_BL,S_WL,
        S_IS,S_IL,S_IQ,
        S_FS,S_FL,S_FX,S_FV,S_FXX,
        S_MD,
        S_NEAR,S_FAR,S_SHORT,
        S_T,
        S_XMM,
        S_YMM
      );

      TOpSizes = set of topsize;

{*****************************************************************************
                                Registers
*****************************************************************************}
  const
      {# Standard opcode string table (for each tasmop enumeration). The
         opcode strings should conform to the names as defined by the
         processor manufacturer.
      }
      std_op2str:op2strtable={$i i386int.inc}

{*****************************************************************************
                               GDB Information
*****************************************************************************}

      {# Register indexes for stabs information, when some
         parameters or variables are stored in registers.

         Taken from i386.c (dbx_register_map) and i386.h
          (FIXED_REGISTERS) from GCC 3.x source code

      }
      reg_stab_table : array[tregisterindex] of shortint = (
        {$i r386stab.inc}
      );


{*****************************************************************************
                          Default generic sizes
*****************************************************************************}

      {# Defines the default address size for a processor, }
      OS_ADDR = OS_32;
      {# the natural int size for a processor,
         has to match osuinttype/ossinttype as initialized in psystem }
      OS_INT = OS_32;
      OS_SINT = OS_S32;
      {# the maximum float size for a processor,           }
      OS_FLOAT = OS_F80;
      {# the size of a vector register for a processor     }
      OS_VECTOR = OS_M128;

{*****************************************************************************
                          Generic Register names
*****************************************************************************}

      {# Stack pointer register }
      NR_STACK_POINTER_REG = NR_ESP;
      RS_STACK_POINTER_REG = RS_ESP;
      {# Frame pointer register }
      RS_FRAME_POINTER_REG = RS_EBP;
      NR_FRAME_POINTER_REG = NR_EBP;
      { Return address for DWARF }
      NR_RETURN_ADDRESS_REG = NR_EIP;
      {# Register for addressing absolute data in a position independant way,
         such as in PIC code. The exact meaning is ABI specific. For
         further information look at GCC source : PIC_OFFSET_TABLE_REGNUM
      }
      NR_PIC_OFFSET_REG = NR_EBX;
      { Results are returned in this register (32-bit values) }
      NR_FUNCTION_RETURN_REG = NR_EAX;
      RS_FUNCTION_RETURN_REG = RS_EAX;
      { Low part of 64bit return value }
      NR_FUNCTION_RETURN64_LOW_REG = NR_EAX;
      RS_FUNCTION_RETURN64_LOW_REG = RS_EAX;
      { High part of 64bit return value }
      NR_FUNCTION_RETURN64_HIGH_REG = NR_EDX;
      RS_FUNCTION_RETURN64_HIGH_REG = RS_EDX;
      { The value returned from a function is available in this register }
      NR_FUNCTION_RESULT_REG = NR_FUNCTION_RETURN_REG;
      RS_FUNCTION_RESULT_REG = RS_FUNCTION_RETURN_REG;
      { The lowh part of 64bit value returned from a function }
      NR_FUNCTION_RESULT64_LOW_REG = NR_FUNCTION_RETURN64_LOW_REG;
      RS_FUNCTION_RESULT64_LOW_REG = RS_FUNCTION_RETURN64_LOW_REG;
      { The high part of 64bit value returned from a function }
      NR_FUNCTION_RESULT64_HIGH_REG = NR_FUNCTION_RETURN64_HIGH_REG;
      RS_FUNCTION_RESULT64_HIGH_REG = RS_FUNCTION_RETURN64_HIGH_REG;

      { WARNING: don't change to R_ST0!! See comments above implementation of }
      { a_loadfpu* methods in rgcpu (JM)                                      }
      NR_fpu_result_reg = NR_ST;
      NR_mm_result_reg = NR_MM0;

      { Offset where the parent framepointer is pushed }
      PARENT_FRAMEPOINTER_OFFSET = 8;

{*****************************************************************************
                       GCC /ABI linking information
*****************************************************************************}

    const
      {# Registers which must be saved when calling a routine declared as
         cppdecl, cdecl, stdcall, safecall, palmossyscall. The registers
         saved should be the ones as defined in the target ABI and / or GCC.

         This value can be deduced from the CALLED_USED_REGISTERS array in the
         GCC source.
      }
<<<<<<< HEAD
      saved_standard_registers : array[0..3] of tsuperregister = (RS_EBX,RS_ESI,RS_EDI,RS_EBP);

      saved_address_registers : array[0..0] of tsuperregister = (RS_INVALID);
=======
      saved_standard_registers : array[0..2] of tsuperregister = (RS_EBX,RS_ESI,RS_EDI);

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      saved_mm_registers : array[0..0] of tsuperregister = (RS_INVALID);
      {# Required parameter alignment when calling a routine declared as
         stdcall and cdecl. The alignment value should be the one defined
         by GCC or the target ABI.

         The value of this constant is equal to the constant
         PARM_BOUNDARY / BITS_PER_UNIT in the GCC source.
      }
      std_param_align = 4;
