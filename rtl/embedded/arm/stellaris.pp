{
Register definitions and utility code for stellaris
Preliminary startup code 
Geoffrey Barton 2010 08 01  gjb@periphon.net
based on stm32f103 created by Jeppe Johansen 2009 - jepjoh2@kom.aau.dk
}
unit stellaris;

{$goto on}

interface

type
 TBitvector32 = bitpacked array[0..31] of 0..1;

{$PACKRECORDS 4}
const
PeripheralBase 	= $40000000;
PPBbase			= $E0000fff;
APBbase 		= PeripheralBase;
AHBbase 		= PeripheralBase+$54000;
portAoffset=APBbase+$4000;
portBoffset=APBbase+$5000;
portCoffset=APBbase+$6000;
portDoffset=APBbase+$7000;
portEoffset=APBbase+$24000;
portFoffset=APBbase+$25000;
portGoffset=APBbase+$26000;
portHoffset=APBbase+$27000;
portJoffset=APBbase+$3d000;
sysconoffset=APBbase+$fe000;


type
TgpioPort=record
				    data:array[0..255] of dword;dir,is,ibe,iev,im,ris,mis,icr,
				    afsel:dword;dummy1:array[0..54] of dword;dr2r,dr4r,dr8r,odr,pur,pdr,slr,den,lock,cr,amsel,pctl:dword;
				  end;
Tsyscon=record
				  did0,did1,dc0,res0c,dc1,dc2,dc3,dc4,dc5,dc6,dc7,dc8,borc,res34,res38,res3c,
				  src0,src1,src2,res4c,ris,imc,misc,resc,rcc,pllcfg,res68,gpiohbctl,rcc2,res74,res78,moscctl:dword;res80:array[0..31] of dword;rcgc0,
				  rcgc1,rcgc2,res10,scgc0,scgc1,scgc2,
				  res11,dcgc0,dcgc1,dcgc2,res12c,res130,res134,res138,res13c,res140,dsplpclk,res13,res14,res15,piosccal,
				  i2smclk,res174,res178,res17c,res180,res184,res188,res18c,dc9,res194,res198,res19c,nvmstat:dword;
				end;		

{$ALIGN 4}
var

  PortA			:Tgpioport 	absolute portAoffset;
  PortB			:Tgpioport	absolute portBoffset;
  PortC			:Tgpioport 	absolute portCoffset;
  PortD			:Tgpioport	absolute portDoffset;
  PortE			:Tgpioport 	absolute portEoffset;
  PortF			:Tgpioport	absolute portFoffset;
  PortG			:Tgpioport 	absolute portGoffset;
  PortH			:Tgpioport	absolute portHoffset;
  PortJ			:Tgpioport	absolute portJoffset;
  
  syscon		:Tsyscon	absolute sysconoffset;
  rcgc0			:dword absolute (sysconoffset+$100);
  rcgc1			:dword absolute (sysconoffset+$104);
  rcgc2			:dword absolute (sysconoffset+$108);

var
NMI_Handler,
HardFault_Handler,
MemManage_Handler,
BusFault_Handler,
UsageFault_Handler,
SWI_Handler,
DebugMonitor_Handler,
PendingSV_Handler,
Systick_Handler,UART0intvector: pointer;
	
implementation

var
	_data: record end; external name '_data';
	_edata: record end; external name '_edata';
	_etext: record end; external name '_etext';
	_bss_start: record end; external name '_bss_start';
	_bss_end: record end; external name '_bss_end';
	_stack_top: record end; external name '_stack_top';

procedure PASCALMAIN; external name 'PASCALMAIN';

procedure _FPC_haltproc; assembler; nostackframe; public name '_haltproc';
asm
.Lhalt:
	b .Lhalt
end;

procedure _FPC_start; assembler; nostackframe;
label _start;
asm
	.init
	.align 16
	
	.long _stack_top	 			// First entry in NVIC table is the new stack pointer
	.long _start+1         //gjb changed from stm32f version to avoid invstate error when interrupt fires
	//b   _start					// Reset
	.long _start+1
	//b	 .LNMI_Addr				// Non maskable interrupt. The RCC Clock Security System (CSS) is linked to the NMI vector.
	.long _start+1
	//b	 .LHardFault_Addr		// All class of fault
	.long _start+1
	//b	 .LMemManage_Addr		// Memory management
	.long _start+1
	//b	 .LBusFault_Addr		// Pre-fetch fault, memory access fault
	.long _start+1
	//b	 .LUsageFault_Addr	// Undefined instruction or illegal state
	.long _start+1
	//nop							// Reserved
	.long _start+1
	//nop							// Reserved
	.long _start+1
	//nop							// Reserved
	.long _start+1
	//nop							// Reserved
	.long _start+1
	//b	 .LSWI_Addr				// Software Interrupt vector now SVC
	.long _start+1
	//b	 .LDebugMonitor_Addr	// Debug Monitor
	.long _start+1
	//nop							// Reserved
	.long _start+1
	//b	 .LPendingSV_Addr		//	Pendable request for system service
	.long _start+1
	//b	 .LSystick_Addr		// System tick timer
	//16
	.long .LDefaultHandler+1     //GPIOA  #0
	.long .LDefaultHandler+1     //GPIOB
	.long .LDefaultHandler+1     //GPIOC
	.long .LDefaultHandler+1     //GPIOD
	.long .LDefaultHandler+1     //GPIOE
	.long .LUART0handler+1       //.LDefaultHandler+1     //UART0
	.long .LDefaultHandler+1     //UART1
	.long .LDefaultHandler+1     //SSI0
	//24
	.long .LDefaultHandler+1     //I2C0   #8
	.long .LDefaultHandler+1     //PWMF
	.long .LDefaultHandler+1     //PWMG0
	.long .LDefaultHandler+1     //PWMG1
	.long .LDefaultHandler+1     //PWMG2
	.long .LDefaultHandler+1     //QEI0
	.long .LDefaultHandler+1     //ADC0S0
	.long .LDefaultHandler+1     //ADC0S1
	//32
	.long .LDefaultHandler+1     //ADC0S2 #16
	.long .LDefaultHandler+1     //ADC0S3
	.long .LDefaultHandler+1     //WDGTimer01
	.long .LDefaultHandler+1     //T0A
	.long .LDefaultHandler+1     //T0B
	.long .LDefaultHandler+1     //T1A
	.long .LDefaultHandler+1     //T1B
	.long .LDefaultHandler+1     //T2A
	//40
	.long .LDefaultHandler+1     //T2B    #24
	.long .LDefaultHandler+1     //COMP0
	.long .LDefaultHandler+1     //COMP1
	.long .LDefaultHandler+1     //COMP2
	.long .LDefaultHandler+1     //SYSCON
	.long .LDefaultHandler+1     //FLASH
	.long .LDefaultHandler+1     //GPIOF
	.long .LDefaultHandler+1     //GPIOG
	//48
	.long .LDefaultHandler+1     //GPIOH  #32
	.long .LDefaultHandler+1     //UART2
	.long .LDefaultHandler+1     //SSI1
	.long .LDefaultHandler+1     //T3A
	.long .LDefaultHandler+1     //T3B
	.long .LDefaultHandler+1     //I2C1
	.long .LDefaultHandler+1     //QEI1
	.long .LDefaultHandler+1     //CAN0
	//56
	.long .LDefaultHandler+1     //CAN1   #40
	.long .LDefaultHandler+1     //res
	.long .LDefaultHandler+1     //ETH
	.long .LDefaultHandler+1     //res
	.long .LDefaultHandler+1     //USB
	.long .LDefaultHandler+1     //PWMG3
	.long .LDefaultHandler+1     //UDMAS
	.long .LDefaultHandler+1     //UDMAE
	//64
	.long .LDefaultHandler+1     //ADC1S0 #48
	.long .LDefaultHandler+1     //ADC1S1
	.long .LDefaultHandler+1     //ADC1S2
	.long .LDefaultHandler+1     //ADC1S3
	.long .LDefaultHandler+1     //I2S0
	.long .LDefaultHandler+1     //EPI
	.long .LDefaultHandler+1     //GPIOJ
	.long .LDefaultHandler+1     //res    #55

.LNMI_Addr:
	ldr r0,.L1
	ldr pc,[r0]
.LHardFault_Addr:
	ldr r0,.L2
	ldr pc,[r0]
.LMemManage_Addr:
	ldr r0,.L3
	ldr pc,[r0]
.LBusFault_Addr:
	ldr r0,.L4
	ldr pc,[r0]
.LUsageFault_Addr:
	ldr r0,.L5
	ldr pc,[r0]
.LSWI_Addr:
	ldr r0,.L6
	ldr pc,[r0]
.LDebugMonitor_Addr:
	ldr r0,.L7
	ldr pc,[r0]
.LPendingSV_Addr:
	ldr r0,.L8
	ldr pc,[r0]
.LSystick_Addr:
	ldr r0,.L9
	ldr pc,[r0]
.LUART0handler:
  ldr r0,.L10
  ldr pc,[r0]
.L1:
	.long NMI_Handler
.L2:
	.long HardFault_Handler
.L3:
	.long MemManage_Handler
.L4:
	.long BusFault_Handler
.L5:
	.long UsageFault_Handler
.L6:
	.long SWI_Handler
.L7:
	.long DebugMonitor_Handler
.L8:
	.long PendingSV_Handler
.L9:
	.long Systick_Handler   
.L10:
  .long UART0IntVector
  
	.globl _start
	.text
_start:
	
	// Copy initialized data to ram
	ldr r1,.L_etext
	ldr r2,.L_data
	ldr r3,.L_edata
.Lcopyloop:
	cmp r2,r3
	ittt ls
	ldrls r0,[r1],#4
	strls r0,[r2],#4
	bls .Lcopyloop

	// clear onboard ram
	ldr r1,.L_bss_start
	ldr r2,.L_bss_end
	mov r0,#0
.Lzeroloop:
	cmp r1,r2
	itt ls
	strls r0,[r1],#4
	bls .Lzeroloop

	b PASCALMAIN
	b _FPC_haltproc

.L_bss_start:
	.long _bss_start
.L_bss_end:
	.long _bss_end
.L_etext:
	.long _etext
.L_data:
	.long _data
.L_edata:
	.long _edata
.LDefaultHandlerAddr:
	.long .LDefaultHandler
	// default irq handler just returns
.LDefaultHandler:
	mov pc,r14
end;

end.

