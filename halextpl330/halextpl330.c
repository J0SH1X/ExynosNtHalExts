// HAL extension for ARM PL330 DMA controllers (Samsung Exynos: ADMA0 in the
// audio block, PDMA0/1 for the peripherals).
//
// Registers each PL330 the firmware describes in a CSRT resource group
// (VendorId 'SAMS', DeviceId 0x0330 -> OSExtensionDatabase\VEN_SAMS&DEV_0330)
// as a system DMA controller, so drivers get DMA channels through the normal
// Windows DMA API (FixedDMA resources in ACPI, IoGetDmaAdapter, MapTransferEx;
// AutoInitialize for looping audio buffers, ReadDmaCounter for positions).
//
// Per controller the group has one DMA controller descriptor (Type 3,
// Subtype 1, PL330_CSRT_CONTROLLER): base, size, GSI, channel count, the
// system-unique request line range, and per peripheral the FIFO access
// width.  The HAL allocates a non-cached common buffer per channel
// (ReportCommonBuffer); ProgramChannel writes the channel's PL330 program
// there and starts it with DMAGO (non-secure) through the debug interface.
//
// The DMA extension interface is version 1 as Windows 8 RTM introduced it
// (WDK nthalext.h: DMA_INITIALIZATION_BLOCK with SOC_INITIALIZATION_HEADER,
// ten operations).  The DDK headers in this directory are the older M2 ones,
// whose DMA definitions predate that, so the version-1 layouts are declared
// here under their own names.
//
// Program body per transfer word as Linux arch/arm/common/pl330.c of the
// Samsung kernel emits it (_ldst_devtomem / _ldst_memtodev with single
// requests).

#include "halext.h"
#include <arm_acle.h>

#ifndef LOAD_TEST
#define LOAD_TEST 0
#endif
#define PL330_LOAD_TEST_BUGCHECK 0x504C3330   // "PL30"

// ---------------------------------------------------------------------------
// DMA extension interface, version 1 (WDK nthalext.h).
// ---------------------------------------------------------------------------

typedef struct _SG_ELEMENT_V1 {
	PHYSICAL_ADDRESS Address;
	ULONG Length;
	ULONG_PTR Reserved;
} SG_ELEMENT_V1, *PSG_ELEMENT_V1;

typedef struct _SG_LIST_V1 {
	ULONG NumberOfElements;
	ULONG_PTR Reserved;
	SG_ELEMENT_V1 Elements[1];
} SG_LIST_V1, *PSG_LIST_V1;

typedef enum _DMA_INTERRUPT_TYPE_V1 {
	DmaInterruptCompletion,
	DmaInterruptError
} DMA_INTERRUPT_TYPE_V1;

typedef struct _DMA_BINDING_V1 {
	ULONG RequestLine;
	ULONG ChannelNumber;
} DMA_BINDING_V1, *PDMA_BINDING_V1;

typedef struct _DMA_FUNCTION_TABLE_V1 {
	VOID (*InitializeController)(PVOID Context);
	BOOLEAN (*ValidateRequestLineBinding)(PVOID Context, PDMA_BINDING_V1 Binding);
	ULONG (*QueryMaxFragments)(PVOID Context, ULONG Channel, ULONG MaxFragmentsRequested);
	VOID (*ProgramChannel)(PVOID Context, ULONG Channel, ULONG RequestLine, PSG_LIST_V1 Memory,
	                       PHYSICAL_ADDRESS DeviceAddress, BOOLEAN WriteToDevice, BOOLEAN LoopTransfer);
	NTSTATUS (*ConfigureChannel)(PVOID Context, ULONG Channel, ULONG FunctionNumber, PVOID FunctionContext);
	VOID (*FlushChannel)(PVOID Context, ULONG Channel);
	BOOLEAN (*HandleInterrupt)(PVOID Context, PULONG Channel, DMA_INTERRUPT_TYPE_V1* Type);
	ULONG (*ReadDmaCounter)(PVOID Context, ULONG Channel);
	VOID (*ReportCommonBuffer)(PVOID Context, ULONG Channel, PVOID VirtualAddress, PHYSICAL_ADDRESS LogicalAddress);
	BOOLEAN (*CancelTransfer)(PVOID Context, ULONG Channel);
} DMA_FUNCTION_TABLE_V1;

typedef struct _DMA_INIT_BLOCK_V1 {
	SOC_INITIALIZATION_HEADER Header;
	ULONG ControllerId;
	ULONG ChannelCount;
	BOOLEAN CacheCoherent;
	ULONG DmaAddressWidth;
	ULONG SupportedPortWidths;
	ULONG MinimumTransferUnit;
	ULONG MinimumRequestLine;
	ULONG MaximumRequestLine;
	BOOLEAN GeneratesInterrupt;
	LONG Gsi;
	KINTERRUPT_POLARITY InterruptPolarity;
	KINTERRUPT_MODE InterruptMode;
	PVOID InternalData;
	ULONG InternalDataSize;
	DMA_FUNCTION_TABLE_V1* Operations;
} DMA_INIT_BLOCK_V1;

typedef struct _DMA_CHANNEL_INIT_BLOCK_V1 {
	SOC_INITIALIZATION_HEADER Header;
	ULONG ControllerId;
	BOOLEAN GeneratesInterrupt;
	LONG Gsi;
	KINTERRUPT_POLARITY InterruptPolarity;
	KINTERRUPT_MODE InterruptMode;
	ULONG CommonBufferLength;
	ULONG ChannelNumber;
} DMA_CHANNEL_INIT_BLOCK_V1;

#define DMA_PORT_WIDTH_ALL (1 | 2 | 4 | 8 | 16)

// ---------------------------------------------------------------------------
// CSRT contents, as the firmware lays them out (Exynos5420Pkg Csrt.aslc).
// ---------------------------------------------------------------------------

#define PL330_MAX_CHANNELS    8
#define PL330_MAX_PERIPHERALS 32
#define PL330_MAX_FRAGMENTS   16
#define PL330_PROGRAM_BYTES   1024

#pragma pack(push, 1)
typedef struct _PL330_CSRT_SHARED_INFO {
	ULONG Version;                          // 1
	ULONG OsLogBase;                        // 0: no log ring
	ULONG OsLogSize;
} PL330_CSRT_SHARED_INFO, *PPL330_CSRT_SHARED_INFO;

typedef struct _PL330_CSRT_CONTROLLER {
	CSRT_RESOURCE_DESCRIPTOR_HEADER Header; // Type 3 (DMA), Subtype 1 (controller), Uid = controller id
	ULONG BaseAddress;                      // registers (physical)
	ULONG Size;
	ULONG Gsi;                              // the controller's (combined) interrupt
	ULONG ChannelCount;                     // channels to expose (<= 8)
	ULONG RequestLineBase;                  // system-unique request line of peripheral 0
	ULONG RequestLineCount;                 // peripherals (<= 32)
	UCHAR PeripheralWidth[PL330_MAX_PERIPHERALS]; // bytes per FIFO access (1, 2, 4); 0 = 4
} PL330_CSRT_CONTROLLER, *PPL330_CSRT_CONTROLLER;
#pragma pack(pop)

// ---------------------------------------------------------------------------
// OS log ring (as halextmct).
// ---------------------------------------------------------------------------

#define OS_LOG_MAGIC 0x474C534F

static ULONG s_OsLogBase;
static ULONG s_OsLogSize;
static volatile ULONG* s_OsLog;

static void OsLog(const char* Text) {
	if (s_OsLog == NULL) {
		if ((s_OsLogBase == 0) || (s_OsLogSize <= 16)) return;
		PHYSICAL_ADDRESS Phys;
		Phys.QuadPart = s_OsLogBase;
		s_OsLog = (volatile ULONG*)HalMapIoSpace(Phys, s_OsLogSize, MmNonCached);
		if (s_OsLog == NULL) return;
	}
	const ULONG DataSize = s_OsLogSize - 16;
	if (s_OsLog[0] != OS_LOG_MAGIC) {
		s_OsLog[1] = 0;
		s_OsLog[0] = OS_LOG_MAGIC;
	}
	volatile UCHAR* Data = (volatile UCHAR*)&s_OsLog[4];
	ULONG Head = s_OsLog[1];
	ULONG Index = Head;
	while (Index >= DataSize) Index -= DataSize;
	for (; *Text != 0; Text++, Head++) {
		Data[Index] = (UCHAR)*Text;
		if (++Index == DataSize) Index = 0;
	}
	s_OsLog[1] = Head;
}

static void OsLogHex(const char* Text, ULONG Value) {
	static const char Digits[] = "0123456789abcdef";
	char Buffer[12];
	int Index;

	OsLog(Text);
	Buffer[0] = '0';
	Buffer[1] = 'x';
	for (Index = 0; Index < 8; Index++) {
		Buffer[2 + Index] = Digits[(Value >> (28 - 4 * Index)) & 0xF];
	}
	Buffer[10] = '\n';
	Buffer[11] = 0;
	OsLog(Buffer);
}

// ---------------------------------------------------------------------------
// PL330 registers and instructions (ARM PL330 TRM).
// ---------------------------------------------------------------------------

#define PL330_DSR          0x000
#define PL330_INTEN        0x020
#define PL330_INTSTATUS    0x028
#define PL330_INTCLR       0x02C
#define PL330_FSRC         0x034
#define PL330_CSR(c)       (0x100 + (c) * 8)
#define PL330_SAR(c)       (0x400 + (c) * 0x20)
#define PL330_DAR(c)       (0x404 + (c) * 0x20)
#define PL330_DBGSTATUS    0xD00
#define PL330_DBGCMD       0xD04
#define PL330_DBGINST0     0xD08
#define PL330_DBGINST1     0xD0C
#define PL330_CR0          0xE00

#define DMAEND             0x00
#define DMAKILL            0x01
#define DMALD              0x04
#define DMAST              0x08
#define DMAWMB             0x13
#define DMALP(lc)          (0x20 | ((lc) << 1))
#define DMALDPS            0x25
#define DMASTPS            0x29
#define DMALPFE            0x28
#define DMALPEND(lc)       (0x38 | ((lc) << 2))
#define DMAWFPS            0x30
#define DMASEV             0x34
#define DMAFLUSHP          0x35
#define DMAMOV             0xBC
#define DMAGO_NS           0xA2
#define MOV_SAR            0
#define MOV_CCR            1
#define MOV_DAR            2

// CCR: burst size = log2(width) on both sides, one beat per burst, non-secure.
#define CCR_SRC_INC        (1u << 0)
#define CCR_SRC_SIZE(s)    ((ULONG)(s) << 1)
#define CCR_SRC_NS         (1u << 9)
#define CCR_DST_INC        (1u << 14)
#define CCR_DST_SIZE(s)    ((ULONG)(s) << 15)
#define CCR_DST_NS         (1u << 23)

// ---------------------------------------------------------------------------
// Controller state: copied by the HAL into its own storage, handed back as
// the context of every operation.
// ---------------------------------------------------------------------------

typedef struct _PL330_CHANNEL {
	volatile UCHAR* Program;         // common buffer (non-cached)
	ULONG ProgramPhys;
	BOOLEAN Active;
	BOOLEAN ToDevice;
	BOOLEAN Loop;
	ULONG Fragments;
	ULONG FragmentAddress[PL330_MAX_FRAGMENTS];
	ULONG FragmentLength[PL330_MAX_FRAGMENTS];
	ULONG TotalLength;
} PL330_CHANNEL;

typedef struct _PL330_CONTROLLER {
	ULONG Phys;
	ULONG Size;
	volatile UCHAR* Base;
	BOOLEAN Ready;                   // reset + interrupts on (first ProgramChannel)
	ULONG Id;
	ULONG ChannelCount;
	ULONG RequestLineBase;
	ULONG RequestLineCount;
	UCHAR PeripheralWidth[PL330_MAX_PERIPHERALS];
	PL330_CHANNEL Channel[PL330_MAX_CHANNELS];
} PL330_CONTROLLER, *PPL330_CONTROLLER;

static ULONG Rd(PPL330_CONTROLLER C, ULONG Offset) {
	return *(volatile ULONG*)(C->Base + Offset);
}

static void Wr(PPL330_CONTROLLER C, ULONG Offset, ULONG Value) {
	*(volatile ULONG*)(C->Base + Offset) = Value;
}

static BOOLEAN DebugInstruction(PPL330_CONTROLLER C, ULONG Inst0, ULONG Inst1) {
	ULONG Try;

	for (Try = 0; (Try < 100000) && ((Rd(C, PL330_DBGSTATUS) & 1) != 0); Try++) {
	}
	if ((Rd(C, PL330_DBGSTATUS) & 1) != 0) return FALSE;
	Wr(C, PL330_DBGINST0, Inst0);
	Wr(C, PL330_DBGINST1, Inst1);
	Wr(C, PL330_DBGCMD, 0);
	return TRUE;
}

static void KillChannel(PPL330_CONTROLLER C, ULONG Channel) {
	if (!C->Ready) return;
	// DMAKILL on the channel thread: bit 0 = channel thread, bits 10:8 = channel.
	DebugInstruction(C, ((ULONG)DMAKILL << 16) | (Channel << 8) | 1, 0);
	C->Channel[Channel].Active = FALSE;
}

// ---------------------------------------------------------------------------
// Program builder.
// ---------------------------------------------------------------------------

typedef struct _PROGRAM {
	volatile UCHAR* Code;
	ULONG Length;
	BOOLEAN Overflow;
} PROGRAM;

static void Emit(PROGRAM* P, UCHAR Byte) {
	if (P->Length >= PL330_PROGRAM_BYTES) {
		P->Overflow = TRUE;
		return;
	}
	P->Code[P->Length++] = Byte;
}

static void EmitMov(PROGRAM* P, UCHAR Register, ULONG Value) {
	Emit(P, DMAMOV);
	Emit(P, Register);
	Emit(P, (UCHAR)Value);
	Emit(P, (UCHAR)(Value >> 8));
	Emit(P, (UCHAR)(Value >> 16));
	Emit(P, (UCHAR)(Value >> 24));
}

static void EmitLoopEnd(PROGRAM* P, UCHAR Opcode, ULONG Start) {
	ULONG Jump = P->Length - Start;

	if (Jump > 255) P->Overflow = TRUE;
	Emit(P, Opcode);
	Emit(P, (UCHAR)Jump);
}

// Count transfers of one peripheral access each (single requests).
static void EmitTransfers(PROGRAM* P, ULONG Count, UCHAR Peripheral, BOOLEAN ToDevice) {
	ULONG Outer;
	ULONG Inner;
	ULONG Start;
	ULONG InnerStart;
	UCHAR Peri = (UCHAR)(Peripheral << 3);

	while (Count != 0) {
		// Up to 256 x 256 per block: Outer x 256, or a last run below 256.
		// No division: on ARM that pulls the CRT divide helper (and with it
		// __security_cookie) out of ntoskrnl.lib.
		if (Count >= 256) {
			Inner = 256;
			Outer = Count >> 8;
			if (Outer > 256) Outer = 256;
		} else {
			Inner = Count;
			Outer = 1;
		}

		Emit(P, DMALP(0));
		Emit(P, (UCHAR)(Outer - 1));
		Start = P->Length;
		Emit(P, DMALP(1));
		Emit(P, (UCHAR)(Inner - 1));
		InnerStart = P->Length;
		Emit(P, DMAWFPS);
		Emit(P, Peri);
		if (ToDevice) {
			Emit(P, DMALD);
			Emit(P, DMASTPS);
			Emit(P, Peri);
		} else {
			Emit(P, DMALDPS);
			Emit(P, Peri);
			Emit(P, DMAST);
		}
		Emit(P, DMAFLUSHP);
		Emit(P, Peri);
		EmitLoopEnd(P, DMALPEND(1), InnerStart);
		EmitLoopEnd(P, DMALPEND(0), Start);
		Count -= Outer * Inner;
	}
}

static ULONG WidthShift(ULONG Width) {
	return (Width == 1) ? 0 : (Width == 2) ? 1 : 2;
}

// ---------------------------------------------------------------------------
// DMA operations.
// ---------------------------------------------------------------------------

// Map only: a controller may sit in a power domain that is off at boot (ADMA0
// is in the audio block, MAU); its registers are first touched once a driver
// programs a channel, i.e. after it powered its block (ControllerReady).
static VOID Pl330InitializeController(PVOID Context) {
	PPL330_CONTROLLER C = (PPL330_CONTROLLER)Context;
	PHYSICAL_ADDRESS Phys;

	if (C->Base == NULL) {
		Phys.QuadPart = C->Phys;
		C->Base = (volatile UCHAR*)HalMapIoSpace(Phys, C->Size, MmNonCached);
	}
	if (C->Base == NULL) {
		OsLog("halextpl330: cannot map the controller\n");
	}
}

static BOOLEAN ControllerReady(PPL330_CONTROLLER C) {
	ULONG Channel;

	if (C->Ready) return TRUE;
	if (C->Base == NULL) return FALSE;
	C->Ready = TRUE;
	for (Channel = 0; Channel < C->ChannelCount; Channel++) {
		KillChannel(C, Channel);
	}
	// Event n = channel n raises the interrupt line.
	Wr(C, PL330_INTCLR, 0xFFFFFFFF);
	Wr(C, PL330_INTEN, (1u << C->ChannelCount) - 1);
	OsLogHex("halextpl330: controller up, CR0 ", Rd(C, PL330_CR0));
	return TRUE;
}

static BOOLEAN Pl330ValidateRequestLineBinding(PVOID Context, PDMA_BINDING_V1 Binding) {
	PPL330_CONTROLLER C = (PPL330_CONTROLLER)Context;

	return (Binding->RequestLine >= C->RequestLineBase) &&
	       (Binding->RequestLine < C->RequestLineBase + C->RequestLineCount) &&
	       (Binding->ChannelNumber < C->ChannelCount);
}

static ULONG Pl330QueryMaxFragments(PVOID Context, ULONG Channel, ULONG MaxFragmentsRequested) {
	UNREFERENCED_PARAMETER(Context);
	UNREFERENCED_PARAMETER(Channel);
	return (MaxFragmentsRequested < PL330_MAX_FRAGMENTS) ? MaxFragmentsRequested : PL330_MAX_FRAGMENTS;
}

static VOID Pl330ProgramChannel(PVOID Context, ULONG Channel, ULONG RequestLine, PSG_LIST_V1 Memory,
                                PHYSICAL_ADDRESS DeviceAddress, BOOLEAN WriteToDevice, BOOLEAN LoopTransfer) {
	PPL330_CONTROLLER C = (PPL330_CONTROLLER)Context;
	PL330_CHANNEL* Ch;
	PROGRAM P;
	ULONG Peripheral;
	ULONG Width;
	ULONG Shift;
	ULONG Index;
	ULONG LoopStart;
	ULONG Ccr;

	if ((Channel >= C->ChannelCount) || !ControllerReady(C)) return;
	Ch = &C->Channel[Channel];
	if ((Ch->Program == NULL) || (Memory == NULL) || (Memory->NumberOfElements == 0) ||
	    (Memory->NumberOfElements > PL330_MAX_FRAGMENTS)) {
		OsLog("halextpl330: ProgramChannel without program buffer or with too many fragments\n");
		return;
	}
	KillChannel(C, Channel);

	Peripheral = RequestLine - C->RequestLineBase;
	Width = (Peripheral < PL330_MAX_PERIPHERALS) ? C->PeripheralWidth[Peripheral] : 4;
	if ((Width != 1) && (Width != 2) && (Width != 4)) Width = 4;
	Shift = WidthShift(Width);

	Ch->ToDevice = WriteToDevice;
	Ch->Loop = LoopTransfer;
	Ch->Fragments = Memory->NumberOfElements;
	Ch->TotalLength = 0;
	for (Index = 0; Index < Ch->Fragments; Index++) {
		Ch->FragmentAddress[Index] = Memory->Elements[Index].Address.LowPart;
		Ch->FragmentLength[Index] = Memory->Elements[Index].Length & ~(Width - 1);
		Ch->TotalLength += Ch->FragmentLength[Index];
	}

	P.Code = Ch->Program;
	P.Length = 0;
	P.Overflow = FALSE;
	Ccr = CCR_SRC_SIZE(Shift) | CCR_DST_SIZE(Shift) | CCR_SRC_NS | CCR_DST_NS |
	      (WriteToDevice ? CCR_SRC_INC : CCR_DST_INC);
	EmitMov(&P, MOV_CCR, Ccr);
	EmitMov(&P, WriteToDevice ? MOV_DAR : MOV_SAR, DeviceAddress.LowPart);
	Emit(&P, DMAFLUSHP);
	Emit(&P, (UCHAR)(Peripheral << 3));
	LoopStart = P.Length;
	for (Index = 0; Index < Ch->Fragments; Index++) {
		EmitMov(&P, WriteToDevice ? MOV_SAR : MOV_DAR, Ch->FragmentAddress[Index]);
		EmitTransfers(&P, Ch->FragmentLength[Index] >> Shift, (UCHAR)Peripheral, WriteToDevice);
	}
	Emit(&P, DMAWMB);
	Emit(&P, DMASEV);
	Emit(&P, (UCHAR)(Channel << 3));
	if (LoopTransfer) {
		EmitLoopEnd(&P, DMALPFE, LoopStart);
	}
	Emit(&P, DMAEND);
	if (P.Overflow) {
		OsLog("halextpl330: transfer does not fit one program (loop body over 255 bytes?)\n");
		return;
	}

	// The program buffer is non-cached; order the stores before DMAGO.
	__dmb(0xF);
	if (!DebugInstruction(C, (Channel << 24) | ((ULONG)DMAGO_NS << 16), Ch->ProgramPhys)) {
		OsLog("halextpl330: debug interface busy\n");
		return;
	}
	Ch->Active = TRUE;
}

static NTSTATUS Pl330ConfigureChannel(PVOID Context, ULONG Channel, ULONG FunctionNumber, PVOID FunctionContext) {
	UNREFERENCED_PARAMETER(Context);
	UNREFERENCED_PARAMETER(Channel);
	UNREFERENCED_PARAMETER(FunctionNumber);
	UNREFERENCED_PARAMETER(FunctionContext);
	return STATUS_NOT_SUPPORTED;
}

static VOID Pl330FlushChannel(PVOID Context, ULONG Channel) {
	PPL330_CONTROLLER C = (PPL330_CONTROLLER)Context;

	if ((Channel < C->ChannelCount) && C->Ready) {
		KillChannel(C, Channel);
		Wr(C, PL330_INTCLR, 1u << Channel);
	}
}

static BOOLEAN Pl330HandleInterrupt(PVOID Context, PULONG Channel, DMA_INTERRUPT_TYPE_V1* Type) {
	PPL330_CONTROLLER C = (PPL330_CONTROLLER)Context;
	ULONG Pending;
	ULONG Faults;
	ULONG Index;

	if (!C->Ready) return FALSE;

	// Faulted channels first (FSRC), then completion events (INTSTATUS).
	Faults = Rd(C, PL330_FSRC) & ((1u << C->ChannelCount) - 1);
	Pending = Rd(C, PL330_INTSTATUS) & ((1u << C->ChannelCount) - 1);
	for (Index = 0; Index < C->ChannelCount; Index++) {
		if ((Faults & (1u << Index)) != 0) {
			KillChannel(C, Index);
			*Channel = Index;
			*Type = DmaInterruptError;
			return TRUE;
		}
	}
	for (Index = 0; Index < C->ChannelCount; Index++) {
		if ((Pending & (1u << Index)) != 0) {
			Wr(C, PL330_INTCLR, 1u << Index);
			if (!C->Channel[Index].Loop) C->Channel[Index].Active = FALSE;
			*Channel = Index;
			*Type = DmaInterruptCompletion;
			return TRUE;
		}
	}
	return FALSE;
}

static ULONG Pl330ReadDmaCounter(PVOID Context, ULONG Channel) {
	PPL330_CONTROLLER C = (PPL330_CONTROLLER)Context;
	PL330_CHANNEL* Ch;
	ULONG Address;
	ULONG Done;
	ULONG Index;

	if (!C->Ready || (Channel >= C->ChannelCount)) return 0;
	Ch = &C->Channel[Channel];
	if (!Ch->Active || ((Rd(C, PL330_CSR(Channel)) & 0xF) == 0)) return 0;

	// Bytes left in this pass: find the fragment the memory side is in.
	Address = Rd(C, Ch->ToDevice ? PL330_SAR(Channel) : PL330_DAR(Channel));
	for (Done = 0, Index = 0; Index < Ch->Fragments; Done += Ch->FragmentLength[Index], Index++) {
		if ((Address >= Ch->FragmentAddress[Index]) &&
		    (Address < Ch->FragmentAddress[Index] + Ch->FragmentLength[Index])) {
			return Ch->TotalLength - (Done + (Address - Ch->FragmentAddress[Index]));
		}
	}
	// Between fragments (address register just past one): count it as done.
	for (Done = 0, Index = 0; Index < Ch->Fragments; Index++) {
		Done += Ch->FragmentLength[Index];
		if (Address == Ch->FragmentAddress[Index] + Ch->FragmentLength[Index]) {
			return Ch->TotalLength - Done;
		}
	}
	return Ch->TotalLength;
}

static VOID Pl330ReportCommonBuffer(PVOID Context, ULONG Channel, PVOID VirtualAddress, PHYSICAL_ADDRESS LogicalAddress) {
	PPL330_CONTROLLER C = (PPL330_CONTROLLER)Context;

	if (Channel < C->ChannelCount) {
		C->Channel[Channel].Program = (volatile UCHAR*)VirtualAddress;
		C->Channel[Channel].ProgramPhys = LogicalAddress.LowPart;
	}
}

static BOOLEAN Pl330CancelTransfer(PVOID Context, ULONG Channel) {
	PPL330_CONTROLLER C = (PPL330_CONTROLLER)Context;
	BOOLEAN Running;
	BOOLEAN Pending;

	if (!C->Ready || (Channel >= C->ChannelCount)) return FALSE;
	Running = C->Channel[Channel].Active && ((Rd(C, PL330_CSR(Channel)) & 0xF) != 0);
	Pending = (Rd(C, PL330_INTSTATUS) & (1u << Channel)) != 0;
	KillChannel(C, Channel);
	return Running && !Pending;
}

static DMA_FUNCTION_TABLE_V1 s_Operations = {
	Pl330InitializeController,
	Pl330ValidateRequestLineBinding,
	Pl330QueryMaxFragments,
	Pl330ProgramChannel,
	Pl330ConfigureChannel,
	Pl330FlushChannel,
	Pl330HandleInterrupt,
	Pl330ReadDmaCounter,
	Pl330ReportCommonBuffer,
	Pl330CancelTransfer
};

// ---------------------------------------------------------------------------
// Registration.
// ---------------------------------------------------------------------------

NTSTATUS
AddResourceGroup (
    __in ULONG Handle,
    __in PCSRT_RESOURCE_GROUP_HEADER ResourceGroup
    ) {
	PPL330_CSRT_CONTROLLER Desc;
	PL330_CONTROLLER Data;
	DMA_INIT_BLOCK_V1 Controller;
	DMA_CHANNEL_INIT_BLOCK_V1 Channel;
	NTSTATUS Status;
	ULONG Index;
	ULONG Registered = 0;

#if LOAD_TEST
	KeBugCheckEx(PL330_LOAD_TEST_BUGCHECK, ResourceGroup->VendorId, ResourceGroup->DeviceId, Handle, 0);
#endif

	if (ResourceGroup->SharedInfoLength >= sizeof(PL330_CSRT_SHARED_INFO)) {
		PPL330_CSRT_SHARED_INFO Info = (PPL330_CSRT_SHARED_INFO)(ResourceGroup + 1);
		if (Info->Version >= 1) {
			s_OsLogBase = Info->OsLogBase;
			s_OsLogSize = Info->OsLogSize;
		}
	}

	for (Desc = NULL;;) {
		Desc = (PPL330_CSRT_CONTROLLER)GetNextResourceDescriptor(
			Handle, ResourceGroup, (PCSRT_RESOURCE_DESCRIPTOR_HEADER)Desc,
			CSRT_RD_TYPE_DMA, CSRT_RD_SUBTYPE_DMA_CONTROLLER, CSRT_RD_UID_ANY);
		if (Desc == NULL) break;
		if ((Desc->Header.Length < sizeof(PL330_CSRT_CONTROLLER)) || (Desc->BaseAddress == 0) ||
		    (Desc->ChannelCount == 0) || (Desc->ChannelCount > PL330_MAX_CHANNELS) ||
		    (Desc->RequestLineCount == 0) || (Desc->RequestLineCount > PL330_MAX_PERIPHERALS)) {
			OsLog("halextpl330: bad PL330 controller descriptor\n");
			continue;
		}

		RtlZeroMemory(&Data, sizeof(Data));
		Data.Phys = Desc->BaseAddress;
		Data.Size = Desc->Size;
		Data.Id = Desc->Header.Uid;
		Data.ChannelCount = Desc->ChannelCount;
		Data.RequestLineBase = Desc->RequestLineBase;
		Data.RequestLineCount = Desc->RequestLineCount;
		for (Index = 0; Index < PL330_MAX_PERIPHERALS; Index++) {
			Data.PeripheralWidth[Index] = (Desc->PeripheralWidth[Index] != 0) ? Desc->PeripheralWidth[Index] : 4;
		}

		RtlZeroMemory(&Controller, sizeof(Controller));
		Controller.Header.TableVersion = 1;
		Controller.Header.TableSize = sizeof(Controller);
		Controller.ControllerId = Data.Id;
		Controller.ChannelCount = Data.ChannelCount;
		Controller.CacheCoherent = FALSE;
		Controller.DmaAddressWidth = 32;
		Controller.SupportedPortWidths = DMA_PORT_WIDTH_ALL;
		Controller.MinimumTransferUnit = 1;
		Controller.MinimumRequestLine = Data.RequestLineBase;
		Controller.MaximumRequestLine = Data.RequestLineBase + Data.RequestLineCount - 1;
		Controller.GeneratesInterrupt = TRUE;
		Controller.Gsi = (LONG)Desc->Gsi;
		Controller.InterruptPolarity = InterruptActiveHigh;
		Controller.InterruptMode = LevelSensitive;
		Controller.InternalData = &Data;
		Controller.InternalDataSize = sizeof(Data);
		Controller.Operations = &s_Operations;
		Status = HextRegisterDmaController(Handle, ResourceGroup, (PVOID)&Controller);
		if (!NT_SUCCESS(Status)) {
			OsLogHex("halextpl330: controller registration failed ", (ULONG)Status);
			continue;
		}

		for (Index = 0; Index < Data.ChannelCount; Index++) {
			RtlZeroMemory(&Channel, sizeof(Channel));
			Channel.Header.TableVersion = 1;
			Channel.Header.TableSize = sizeof(Channel);
			Channel.ControllerId = Data.Id;
			Channel.GeneratesInterrupt = FALSE;      // the controller's line covers all channels
			Channel.CommonBufferLength = PL330_PROGRAM_BYTES;
			Channel.ChannelNumber = Index;
			Status = HextRegisterDmaChannel(Handle, ResourceGroup, (PVOID)&Channel);
			if (!NT_SUCCESS(Status)) {
				OsLogHex("halextpl330: channel registration failed ", (ULONG)Status);
			}
		}
		OsLogHex("halextpl330: registered PL330 at ", Data.Phys);
		Registered++;
	}
	return (Registered != 0) ? STATUS_SUCCESS : STATUS_UNSUCCESSFUL;
}
