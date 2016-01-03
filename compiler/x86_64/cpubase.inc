{
    Copyright (c) 1998-2000 by Florian Klaempfl and Peter Vreman

    Contains the basic declarations for the x86-64 architecture

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
{ This include file contains the basic declarations for the x86-64 architecture.
}

{*****************************************************************************
                                Operand Sizes
*****************************************************************************}

type
  topsize = (S_NO,
    S_B,S_W,S_L,S_Q,S_BW,S_BL,S_WL,S_BQ,S_WQ,S_LQ,
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
  { Standard opcode string table (for each tasmop enumeration). The
    opcode strings should conform to the names as defined by the
    processor manufacturer.
  }
  std_op2str:op2strtable={$i x8664int.inc}

{*****************************************************************************
                               GDB Information
*****************************************************************************}

      {# Register indexes for stabs information, when some
         parameters or variables are stored in registers.

         Taken from i386.c (dbx_register_map) and i386.h
          (FIXED_REGISTERS) from GCC 3.x source code

      }
      stab_regindex : array[tregisterindex] of shortint = (
        {$i r8664stab.inc}
      );

{*****************************************************************************
                          Default generic sizes
*****************************************************************************}

      { Defines the default address size for a processor, }
      OS_ADDR = OS_64;
      { the natural int size for a processor,
        has to match osuinttype/ossinttype as initialized in psystem }
      OS_INT = OS_64;
      OS_SINT = OS_S64;
      { the maximum float size for a processor,           }
      OS_FLOAT = OS_F80;
      { the size of a vector register for a processor     }
      OS_VECTOR = OS_M128;

{*****************************************************************************
                          Generic Register names
*****************************************************************************}

      {# Stack pointer register }
      RS_STACK_POINTER_REG = RS_RSP;
      NR_STACK_POINTER_REG = NR_RSP;
      {# Frame pointer register }
      RS_FRAME_POINTER_REG = RS_RBP;
      NR_FRAME_POINTER_REG = NR_RBP;
      { Return address for DWARF }
      NR_RETURN_ADDRESS_REG = NR_EIP;
      { Register for addressing absolute data in a position independant way,
        such as in PIC code. The exact meaning is ABI specific. For
        further information look at GCC source : PIC_OFFSET_TABLE_REGNUM
      }
      NR_PIC_OFFSET_REG = NR_RBX;
      { Results are returned in this register (both 32 and 64 bits }
      NR_FUNCTION_RETURN_REG = NR_RAX;
      RS_FUNCTION_RETURN_REG = RS_RAX;
      NR_FUNCTION_RETURN_REG_HIGH = NR_RDX;
      RS_FUNCTION_RETURN_REG_HIGH = RS_RDX;
      { The value returned from a function is available in this register }
      NR_FUNCTION_RESULT_REG = NR_FUNCTION_RETURN_REG;
      RS_FUNCTION_RESULT_REG = RS_FUNCTION_RETURN_REG;
      NR_FUNCTION_RESULT_REG_HIGH = NR_FUNCTION_RETURN_REG_HIGH;
      RS_FUNCTION_RESULT_REG_HIGH = RS_FUNCTION_RETURN_REG_HIGH;

      { WARNING: don't change to R_ST0!! See comments above implementation of }
      { a_loadfpu* methods in rgcpu (JM)                                      }
      NR_FPU_RESULT_REG = NR_ST;
      NR_MM_RESULT_REG = NR_XMM0;
      RS_MM_RESULT_REG = RS_XMM0;
      NR_MM_RESULT_REG_HIGH = NR_XMM1;
      RS_MM_RESULT_REG_HIGH = RS_XMM1;

      { Offset where the parent framepointer is pushed }
      PARENT_FRAMEPOINTER_OFFSET = 16;

{*****************************************************************************
                       GCC /ABI linking information
*****************************************************************************}

    const
      { these arrays differ between unix and win64 }
      saved_standard_registers : array of tsuperregister = nil;
      saved_mm_registers : array of tsuperregister = nil;
<<<<<<< HEAD
<<<<<<< HEAD

      saved_address_registers : array[0..0] of tsuperregister = (RS_INVALID);
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      { Required parameter alignment when calling a routine declared as
        stdcall and cdecl. The alignment value should be the one defined
        by GCC or the target ABI.

         The value of this constant is equal to the constant
         PARM_BOUNDARY / BITS_PER_UNIT in the GCC source.
      }
      std_param_align = 8;
