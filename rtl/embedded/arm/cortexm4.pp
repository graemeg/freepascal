{
 System register definitions and utility code for Cortex-M4

 Created by Jeppe Johansen 2012 - jeppe@j-software.dk
}
unit cortexm4;

interface

{$PACKRECORDS 2}
const
 SCS_BASE   = $E000E000;
 DWT_BASE   = $E0001000;
 FP_BASE    = $E0002000;
 ITM_BASE   = $E0000000;
 TPIU_BASE  = $E0040000;
 ETM_BASE   = $E0041000;

type

 TNVICRegisters = record
  ISER: array[0..7] of longword;
   reserved0: array[0..23] of longword;
  ICER: array[0..7] of longword;
   reserved1: array[0..23] of longword;
  ISPR: array[0..7] of longword;
   reserved2: array[0..23] of longword;
  ICPR: array[0..7] of longword;
   reserved3: array[0..23] of longword;
  IABR: array[0..7] of longword;
   reserved4: array[0..55] of longword;
  IP: array[0..239] of byte;
   reserved5: array[0..643] of longword;
  STIR: longword;
 end;

 TSCBRegisters = record
  CPUID,                            {!< CPU ID Base Register                                     }
  ICSR,                             {!< Interrupt Control State Register                         }
  VTOR,                             {!< Vector Table Offset Register                             }
  AIRCR,                            {!< Application Interrupt / Reset Control Register           }
  SCR,                              {!< System Control Register                                  }
  CCR: longword;                    {!< Configuration Control Register                           }
  SHP: array[0..11] of byte;        {!< System Handlers Priority Registers (4-7, 8-11, 12-15)    }
  SHCSR,                            {!< System Handler Control and State Register                }
  CFSR,                             {!< Configurable Fault Status Register                       }
  HFSR,                             {!< Hard Fault Status Register                               }
  DFSR,                             {!< Debug Fault Status Register                              }
  MMFAR,                            {!< Mem Manage Address Register                              }
  BFAR,                             {!< Bus Fault Address Register                               }
  AFSR: longword;                   {!< Auxiliary Fault Status Register                          }
  // CPUID registers
  PFR: array[0..1] of longword;     {!< Processor Feature Register                               }
  DFR,                              {!< Debug Feature Register                                   }
  ADR: longword;                    {!< Auxiliary Feature Register                               }
  MMFR: array[0..3] of longword;    {!< Memory Model Feature Register                            }
  ISAR: array[0..5] of longword;    {!< ISA Feature Register                                     }
  CLIDR,
  CTR,
  CCSIDR,
  CSSELR: longword;

  CPACR: longword;
 end;

 TSysTickRegisters = record
  Ctrl,
  Load,
  Val,
  Calib: longword;
 end;

 TIDRegisters = record
  PID4_7: array[0..3] of longword;
  PID0_3: array[0..3] of longword;
  CID: array[0..3] of longword;
 end;

 TCoreDebugRegisters = record
  DHCSR,
  DCRSR,
  DCRDR,
  DEMCR: longword;
 end;

 TFPRegisters = record
  Ctrl,
  Remap: longword;
  Comp: array[0..7] of longword;
  res: array[0..987] of longword;
  ID: TIDRegisters;
 end;

 TDWTEntry = record
  Comp,
  Mask,
  Func,
  res: longword;
 end;

 TDWTRegisters = record
  Ctrl,
  CycCnt,
  CPICnt,
  ExcCnt,
  SleepCnt,
  LSUCnt,
  FoldCnt,
  PCSR: longword;
  Entries: array[0..3] of TDWTEntry;
 end;

 TITMRegisters = record
  Stimulus: array[0..31] of longword;
   res0: array[0..($E00-$7C-4)-1] of byte;
  TraceEnable: longword;
   res1: array[0..($E40-$E00-4)-1] of byte;
  TracePrivilege: longword;
   res2: array[0..($E80-$E40-4)-1] of byte;
  TraceControl: longword;
   res3: array[0..($EF8-$E80-4)-1] of byte;
  IntegrationWrite,
  IntegrationRead,
  IntegrationModeCtrl: longword;
   res4: array[0..($FB0-$F00-4)-1] of byte;
  LockAccess,
  LockStatus: longword;
   res5: array[0..($FD0-$FB4-4)-1] of byte;
  ID: TIDRegisters;
 end;

 TTPIURegisters = record
  SupportedSyncPortSizes,
  CurrentSyncPortSize: longword;
   res0: array[0..($10-$04-4)-1] of byte;
  AsyncColckPrescaler: longword;
   res1: array[0..($F0-$10-4)-1] of byte;
  SelectedPinProtocol: longword;
   res2: array[0..($100-$F0-4)-1] of byte;
  TriggerControl: array[0..2] of longword;
   res3: array[0..($200-$108-4)-1] of byte;
  TestPattern: array[0..2] of longword;
   res4: array[0..($300-$208-4)-1] of byte;
  FormatFlushStatus,
  FormatControl,
  FormatSyncCounter: longword;
   res5: array[0..($EF0-$308-4)-1] of byte;
  ITATBCTR2: longword;
   res6: longword;
  ITATBCTR0: longword;
 end;

 TFPExtRegisters = record
  FPCCR,
  FPCAR,
  FPDSCR: longword;
  MVFR: array[0..2] of longword;
 end;

var
 // System Control
 InterruptControlType: longword     absolute (SCS_BASE+$0004);
 SCB: TSCBRegisters                 absolute (SCS_BASE+$0D00);
 SysTick: TSysTickRegisters         absolute (SCS_BASE+$0010);
 NVIC: TNVICRegisters               absolute (SCS_BASE+$0100);
 SoftwareTriggerInterrupt: longword absolute (SCS_BASE+$0000);
 FPExt: TFPExtRegisters             absolute (SCS_BASE+$0F34);

 // Core Debug
 CoreDebug: TCoreDebugRegisters     absolute (SCS_BASE+$0DF0);

 // Flash Patch
 FP: TFPRegisters                   absolute FP_BASE;

 DWT: TDWTRegisters                 absolute DWT_BASE;

 ITM: TITMRegisters                 absolute ITM_BASE;

 TPIU: TTPIURegisters               absolute TPIU_BASE;

type
 TITM_Port = 0..31;

procedure ITM_SendData(Port: TITM_Port; Data: longword); inline;

implementation

const
 CoreDebug_DEMCR_TRCENA = $01000000;
 ITM_TCR_ITMENA   = $00000001;

procedure ITM_SendData(Port: TITM_Port; Data: longword);
begin
   if ((CoreDebug.DEMCR and CoreDebug_DEMCR_TRCENA) <> 0) and
      ((itm.TraceControl and ITM_TCR_ITMENA) <> 0) and
      ((ITM.TraceEnable and (1 shl Port)) <> 0) then
   begin
      while ITM.Stimulus[Port] = 0 do;
      ITM.Stimulus[Port] := Data;
   end;
end;

end.
