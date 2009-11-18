/*** m6800: Portable 6800 class  emulator *************************************

    m6800.c

    References:

        6809 Simulator V09, By L.C. Benschop, Eidnhoven The Netherlands.

        m6809: Portable 6809 emulator, DS (6809 code in MAME, derived from
            the 6809 Simulator V09)

        6809 Microcomputer Programming & Interfacing with Experiments"
            by Andrew C. Staugaard, Jr.; Howard W. Sams & Co., Inc.

    System dependencies:    UINT16 must be 16 bit unsigned int
                            UINT8 must be 8 bit unsigned int
                            UINT32 must be more than 16 bits
                            arrays up to 65536 bytes must be supported
                            machine must be twos complement

History
991031  ZV
    Added NSC-8105 support

990319  HJB
    Fixed wrong LSB/MSB order for push/pull word.
    Subtract .extra_cycles at the beginning/end of the exectuion loops.

990316  HJB
    Renamed to 6800, since that's the basic CPU.
    Added different cycle count tables for M6800/2/8, M6801/3 and HD63701.

990314  HJB
    Also added the M6800 subtype.

990311  HJB
    Added _info functions. Now uses static m6808_Regs struct instead
    of single statics. Changed the 16 bit registers to use the generic
    PAIR union. Registers defined using macros. Split the core into
    four execution loops for M6802, M6803, M6808 and HD63701.
    TST, TSTA and TSTB opcodes reset carry flag.
TODO:
    Verify invalid opcodes for the different CPU types.
    Add proper credits to _info functions.
    Integrate m6808_Flags into the registers (multiple m6808 type CPUs?)

990301  HJB
    Modified the interrupt handling. No more pending interrupt checks.
    WAI opcode saves state, when an interrupt is taken (IRQ or OCI),
    the state is only saved if not already done by WAI.

*****************************************************************************/


//#include "debugger.h"
#include "burnint.h"
#include "m6800.h"

#define VERBOSE 0

#if VERBOSE
#define LOG(x)	logerror x
#else
#define LOG(x)
#endif

#define M6800_INLINE		static
#define change_pc(newpc)	m6800.pc.w.l = (newpc)
#define M6800_CLEAR_LINE	0

#if 0
/* CPU subtypes, needed for extra insn after TAP/CLI/SEI */
enum {
	SUBTYPE_M6800,
	SUBTYPE_M6801,
	SUBTYPE_M6802,
	SUBTYPE_M6803,
	SUBTYPE_M6808,
	SUBTYPE_HD63701,
	SUBTYPE_NSC8105
};
#endif

/* 680x registers */
static m6800_Regs m6800;

#define m6801   m6800
#define m6802   m6800
#define m6803	m6800
#define m6808	m6800
#define hd63701 m6800
#define nsc8105 m6800

#define	pPPC	m6800.ppc
#define pPC 	m6800.pc
#define pS		m6800.s
#define pX		m6800.x
#define pD		m6800.d

#define PC		m6800.pc.w.l
#define PCD		m6800.pc.d
#define S		m6800.s.w.l
#define SD		m6800.s.d
#define X		m6800.x.w.l
#define D		m6800.d.w.l
#define A		m6800.d.b.h
#define B		m6800.d.b.l
#define CC		m6800.cc

#define CT		m6800.counter.w.l
#define CTH		m6800.counter.w.h
#define CTD		m6800.counter.d
#define OC		m6800.output_compare.w.l
#define OCH		m6800.output_compare.w.h
#define OCD		m6800.output_compare.d
#define TOH		m6800.timer_over.w.l
#define TOD		m6800.timer_over.d

static PAIR ea; 		/* effective address */
#define EAD ea.d
#define EA	ea.w.l

/* public globals */
static int m6800_ICount;

/* point of next timer event */
static UINT32 timer_next;

/* DS -- THESE ARE RE-DEFINED IN m6800.h TO RAM, ROM or FUNCTIONS IN cpuintrf.c */
#define RM				M6800_RDMEM
#define WM				M6800_WRMEM
#define M_RDOP			M6800_RDOP
#define M_RDOP_ARG		M6800_RDOP_ARG

/* macros to access memory */
#define IMMBYTE(b)	b = M_RDOP_ARG(PCD); PC++
#define IMMWORD(w)	w.d = (M_RDOP_ARG(PCD)<<8) | M_RDOP_ARG((PCD+1)&0xffff); PC+=2

#define PUSHBYTE(b) WM(SD,b); --S
#define PUSHWORD(w) WM(SD,w.b.l); --S; WM(SD,w.b.h); --S
#define PULLBYTE(b) S++; b = RM(SD)
#define PULLWORD(w) S++; w.d = RM(SD)<<8; S++; w.d |= RM(SD)

#define MODIFIED_tcsr {	\
	m6800.irq2 = (m6800.tcsr&(m6800.tcsr<<3))&(TCSR_ICF|TCSR_OCF|TCSR_TOF); \
}

#define SET_TIMER_EVENT {					\
	timer_next = (OCD - CTD < TOD - CTD) ? OCD : TOD;	\
}

/* cleanup high-word of counters */
#define CLEANUP_conters {						\
	OCH -= CTH;									\
	TOH -= CTH;									\
	CTH = 0;									\
	SET_TIMER_EVENT;							\
}

/* when change freerunningcounter or outputcapture */
#define MODIFIED_counters {						\
	OCH = (OC >= CT) ? CTH : CTH+1;				\
	SET_TIMER_EVENT;							\
}

/* take interrupt */
#define TAKE_ICI ENTER_INTERRUPT("M6800#%d take ICI\n",0xfff6)
#define TAKE_OCI ENTER_INTERRUPT("M6800#%d take OCI\n",0xfff4)
#define TAKE_TOI ENTER_INTERRUPT("M6800#%d take TOI\n",0xfff2)
#define TAKE_SCI ENTER_INTERRUPT("M6800#%d take SCI\n",0xfff0)
#define TAKE_TRAP ENTER_INTERRUPT("M6800#%d take TRAP\n",0xffee)

/* check IRQ2 (internal irq) */
#define CHECK_IRQ2 {											\
	if(m6800.irq2&(TCSR_ICF|TCSR_OCF|TCSR_TOF))					\
	{															\
		if(m6800.irq2&TCSR_ICF)									\
		{														\
			TAKE_ICI;											\
		}														\
		else if(m6800.irq2&TCSR_OCF)							\
		{														\
			TAKE_OCI;											\
		}														\
		else if(m6800.irq2&TCSR_TOF)							\
		{														\
			TAKE_TOI;											\
		}														\
	}															\
}

/* operate one instruction for */
#define ONE_MORE_INSN() {		\
	UINT8 ireg; 							\
	pPPC = pPC; 							\
	ireg=M_RDOP(PCD);						\
	PC++;									\
	(*m6800.insn[ireg])();					\
	INCREMENT_COUNTER(m6800.cycles[ireg]);	\
}

/* check the IRQ lines for pending interrupts */
#define CHECK_IRQ_LINES() {										\
	if( !(CC & 0x10) )											\
	{															\
		if( m6800.irq_state[M6800_IRQ_LINE] != M6800_CLEAR_LINE )		\
		{	/* standard IRQ */									\
			ENTER_INTERRUPT("M6800#%d take IRQ1\n",0xfff8);		\
		}														\
		else													\
			CHECK_IRQ2;											\
	}															\
}

/* CC masks                       HI NZVC
                                7654 3210   */
#define CLR_HNZVC	CC&=0xd0
#define CLR_NZV 	CC&=0xf1
#define CLR_HNZC	CC&=0xd2
#define CLR_NZVC	CC&=0xf0
#define CLR_Z		CC&=0xfb
#define CLR_NZC 	CC&=0xf2
#define CLR_ZC		CC&=0xfa
#define CLR_C		CC&=0xfe

/* macros for CC -- CC bits affected should be reset before calling */
#define SET_Z(a)		if(!(a))SEZ
#define SET_Z8(a)		SET_Z((UINT8)(a))
#define SET_Z16(a)		SET_Z((UINT16)(a))
#define SET_N8(a)		CC|=(((a)&0x80)>>4)
#define SET_N16(a)		CC|=(((a)&0x8000)>>12)
#define SET_H(a,b,r)	CC|=((((a)^(b)^(r))&0x10)<<1)
#define SET_C8(a)		CC|=(((a)&0x100)>>8)
#define SET_C16(a)		CC|=(((a)&0x10000)>>16)
#define SET_V8(a,b,r)	CC|=((((a)^(b)^(r)^((r)>>1))&0x80)>>6)
#define SET_V16(a,b,r)	CC|=((((a)^(b)^(r)^((r)>>1))&0x8000)>>14)

static const UINT8 flags8i[256]=	 /* increment */
{
0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x0a,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,
0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,
0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,
0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,
0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,
0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,
0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,
0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08
};
static const UINT8 flags8d[256]= /* decrement */
{
0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x02,
0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,
0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,
0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,
0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,
0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,
0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,
0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,
0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08
};
#define SET_FLAGS8I(a)		{CC|=flags8i[(a)&0xff];}
#define SET_FLAGS8D(a)		{CC|=flags8d[(a)&0xff];}

/* combos */
#define SET_NZ8(a)			{SET_N8(a);SET_Z8(a);}
#define SET_NZ16(a)			{SET_N16(a);SET_Z16(a);}
#define SET_FLAGS8(a,b,r)	{SET_N8(r);SET_Z8(r);SET_V8(a,b,r);SET_C8(r);}
#define SET_FLAGS16(a,b,r)	{SET_N16(r);SET_Z16(r);SET_V16(a,b,r);SET_C16(r);}

/* for treating an UINT8 as a signed INT16 */
#define SIGNED(b) ((INT16)(b&0x80?b|0xff00:b))

/* Macros for addressing modes */
#define DIRECT IMMBYTE(EAD)
#define IMM8 EA=PC++
#define IMM16 {EA=PC;PC+=2;}
#define EXTENDED IMMWORD(ea)
#define INDEXED {EA=X+(UINT8)M_RDOP_ARG(PCD);PC++;}

/* macros to set status flags */
#define SEC CC|=0x01
#define CLC CC&=0xfe
#define SEZ CC|=0x04
#define CLZ CC&=0xfb
#define SEN CC|=0x08
#define CLN CC&=0xf7
#define SEV CC|=0x02
#define CLV CC&=0xfd
#define SEH CC|=0x20
#define CLH CC&=0xdf
#define SEI CC|=0x10
#define CLI CC&=~0x10

/* mnemonicos for the Timer Control and Status Register bits */
#define TCSR_OLVL 0x01
#define TCSR_IEDG 0x02
#define TCSR_ETOI 0x04
#define TCSR_EOCI 0x08
#define TCSR_EICI 0x10
#define TCSR_TOF  0x20
#define TCSR_OCF  0x40
#define TCSR_ICF  0x80

#define INCREMENT_COUNTER(amount)	\
{									\
	m6800_ICount -= amount;			\
	CTD += amount;					\
	if( CTD >= timer_next)			\
		check_timer_event();		\
}

#define EAT_CYCLES													\
{																	\
	int cycles_to_eat;												\
																	\
	cycles_to_eat = timer_next - CTD;								\
	if( cycles_to_eat > m6800_ICount) cycles_to_eat = m6800_ICount;	\
	if (cycles_to_eat > 0)											\
	{																\
		INCREMENT_COUNTER(cycles_to_eat);							\
	}																\
}

/* macros for convenience */
#define DIRBYTE(b) {DIRECT;b=RM(EAD);}
#define DIRWORD(w) {DIRECT;w.d=RM16(EAD);}
#define EXTBYTE(b) {EXTENDED;b=RM(EAD);}
#define EXTWORD(w) {EXTENDED;w.d=RM16(EAD);}

#define IDXBYTE(b) {INDEXED;b=RM(EAD);}
#define IDXWORD(w) {INDEXED;w.d=RM16(EAD);}

/* Macros for branch instructions */
#define CHANGE_PC() change_pc(PCD)
#define BRANCH(f) {IMMBYTE(t);if(f){PC+=SIGNED(t);CHANGE_PC();}}
#define NXORV  ((CC&0x08)^((CC&0x02)<<2))

/* Note: we use 99 cycles here for invalid opcodes so that we don't */
/* hang in an infinite loop if we hit one */
static const UINT8 cycles_6800[] =
{
		/* 0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F */
	/*0*/ 99, 2,99,99,99,99, 2, 2, 4, 4, 2, 2, 2, 2, 2, 2,
	/*1*/  2, 2,99,99,99,99, 2, 2,99, 2,99, 2,99,99,99,99,
	/*2*/  4,99, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
	/*3*/  4, 4, 4, 4, 4, 4, 4, 4,99, 5,99,10,99,99, 9,12,
	/*4*/  2,99,99, 2, 2,99, 2, 2, 2, 2, 2,99, 2, 2,99, 2,
	/*5*/  2,99,99, 2, 2,99, 2, 2, 2, 2, 2,99, 2, 2,99, 2,
	/*6*/  7,99,99, 7, 7,99, 7, 7, 7, 7, 7,99, 7, 7, 4, 7,
	/*7*/  6,99,99, 6, 6,99, 6, 6, 6, 6, 6,99, 6, 6, 3, 6,
	/*8*/  2, 2, 2,99, 2, 2, 2,99, 2, 2, 2, 2, 3, 8, 3,99,
	/*9*/  3, 3, 3,99, 3, 3, 3, 4, 3, 3, 3, 3, 4,99, 4, 5,
	/*A*/  5, 5, 5,99, 5, 5, 5, 6, 5, 5, 5, 5, 6, 8, 6, 7,
	/*B*/  4, 4, 4,99, 4, 4, 4, 5, 4, 4, 4, 4, 5, 9, 5, 6,
	/*C*/  2, 2, 2,99, 2, 2, 2,99, 2, 2, 2, 2,99,99, 3,99,
	/*D*/  3, 3, 3,99, 3, 3, 3, 4, 3, 3, 3, 3,99,99, 4, 5,
	/*E*/  5, 5, 5,99, 5, 5, 5, 6, 5, 5, 5, 5,99,99, 6, 7,
	/*F*/  4, 4, 4,99, 4, 4, 4, 5, 4, 4, 4, 4,99,99, 5, 6
};

static const UINT8 cycles_6803[] =
{
		/* 0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F */
	/*0*/ 99, 2,99,99, 3, 3, 2, 2, 3, 3, 2, 2, 2, 2, 2, 2,
	/*1*/  2, 2,99,99,99,99, 2, 2,99, 2,99, 2,99,99,99,99,
	/*2*/  3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
	/*3*/  3, 3, 4, 4, 3, 3, 3, 3, 5, 5, 3,10, 4,10, 9,12,
	/*4*/  2,99,99, 2, 2,99, 2, 2, 2, 2, 2,99, 2, 2,99, 2,
	/*5*/  2,99,99, 2, 2,99, 2, 2, 2, 2, 2,99, 2, 2,99, 2,
	/*6*/  6,99,99, 6, 6,99, 6, 6, 6, 6, 6,99, 6, 6, 3, 6,
	/*7*/  6,99,99, 6, 6,99, 6, 6, 6, 6, 6,99, 6, 6, 3, 6,
	/*8*/  2, 2, 2, 4, 2, 2, 2,99, 2, 2, 2, 2, 4, 6, 3,99,
	/*9*/  3, 3, 3, 5, 3, 3, 3, 3, 3, 3, 3, 3, 5, 5, 4, 4,
	/*A*/  4, 4, 4, 6, 4, 4, 4, 4, 4, 4, 4, 4, 6, 6, 5, 5,
	/*B*/  4, 4, 4, 6, 4, 4, 4, 4, 4, 4, 4, 4, 6, 6, 5, 5,
	/*C*/  2, 2, 2, 4, 2, 2, 2,99, 2, 2, 2, 2, 3,99, 3,99,
	/*D*/  3, 3, 3, 5, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4,
	/*E*/  4, 4, 4, 6, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5,
	/*F*/  4, 4, 4, 6, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5
};

static const UINT8 cycles_63701[] =
{
		/* 0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F */
	/*0*/ 99, 1,99,99, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
	/*1*/  1, 1,99,99,99,99, 1, 1, 2, 2, 4, 1,99,99,99,99,
	/*2*/  3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
	/*3*/  1, 1, 3, 3, 1, 1, 4, 4, 4, 5, 1,10, 5, 7, 9,12,
	/*4*/  1,99,99, 1, 1,99, 1, 1, 1, 1, 1,99, 1, 1,99, 1,
	/*5*/  1,99,99, 1, 1,99, 1, 1, 1, 1, 1,99, 1, 1,99, 1,
	/*6*/  6, 7, 7, 6, 6, 7, 6, 6, 6, 6, 6, 5, 6, 4, 3, 5,
	/*7*/  6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 4, 6, 4, 3, 5,
	/*8*/  2, 2, 2, 3, 2, 2, 2,99, 2, 2, 2, 2, 3, 5, 3,99,
	/*9*/  3, 3, 3, 4, 3, 3, 3, 3, 3, 3, 3, 3, 4, 5, 4, 4,
	/*A*/  4, 4, 4, 5, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5,
	/*B*/  4, 4, 4, 5, 4, 4, 4, 4, 4, 4, 4, 4, 5, 6, 5, 5,
	/*C*/  2, 2, 2, 3, 2, 2, 2,99, 2, 2, 2, 2, 3,99, 3,99,
	/*D*/  3, 3, 3, 4, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4,
	/*E*/  4, 4, 4, 5, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5,
	/*F*/  4, 4, 4, 5, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5
};

static const UINT8 cycles_nsc8105[] =
{
		/* 0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F */
	/*0*/ 99,99, 2,99,99, 2,99, 2, 4, 2, 4, 2, 2, 2, 2, 2,
	/*1*/  2,99, 2,99,99, 2,99, 2,99,99, 2, 2,99,99,99,99,
	/*2*/  4, 4,99, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
	/*3*/  4, 4, 4, 4, 4, 4, 4, 4,99,99, 5,10,99, 9,99,12,
	/*4*/  2,99,99, 2, 2, 2,99, 2, 2, 2, 2,99, 2,99, 2, 2,
	/*5*/  2,99,99, 2, 2, 2,99, 2, 2, 2, 2,99, 2,99, 2, 2,
	/*6*/  7,99,99, 7, 7, 7,99, 7, 7, 7, 7,99, 7, 4, 7, 7,
	/*7*/  6,99,99, 6, 6, 6,99, 6, 6, 6, 6,99, 6, 3, 6, 6,
	/*8*/  2, 2, 2,99, 2, 2, 2,99, 2, 2, 2, 2, 3, 3, 8,99,
	/*9*/  3, 3, 3,99, 3, 3, 3, 4, 3, 3, 3, 3, 4, 4,99, 5,
	/*A*/  5, 5, 5,99, 5, 5, 5, 6, 5, 5, 5, 5, 6, 6, 8, 7,
	/*B*/  4, 4, 4,99, 4, 4, 4, 5, 4, 4, 4, 4, 5, 5, 9, 6,
	/*C*/  2, 2, 2,99, 2, 2, 2,99, 2, 2, 2, 2,99, 3,99,99,
	/*D*/  3, 3, 3,99, 3, 3, 3, 4, 3, 3, 3, 3,99, 4,99, 5,
	/*E*/  5, 5, 5,99, 5, 5, 5, 6, 5, 5, 5, 5, 5, 6,99, 7,
	/*F*/  4, 4, 4,99, 4, 4, 4, 5, 4, 4, 4, 4, 4, 5,99, 6
};

M6800_INLINE UINT32 RM16( UINT32 Addr )
{
	UINT32 result = RM(Addr) << 8;
	return result | RM((Addr+1)&0xffff);
}

M6800_INLINE void WM16( UINT32 Addr, PAIR *p )
{
	WM( Addr, p->b.h );
	WM( (Addr+1)&0xffff, p->b.l );
}

/* IRQ enter */
static void ENTER_INTERRUPT(const char */*message*/,UINT16 irq_vector)
{
//	LOG((message, cpu_getactivecpu()));
	if( m6800.wai_state & (M6800_WAI|M6800_SLP) )
	{
		if( m6800.wai_state & M6800_WAI )
			m6800.extra_cycles += 4;
		m6800.wai_state &= ~(M6800_WAI|M6800_SLP);
	}
	else
	{
		PUSHWORD(pPC);
		PUSHWORD(pX);
		PUSHBYTE(A);
		PUSHBYTE(B);
		PUSHBYTE(CC);
		m6800.extra_cycles += 12;
	}
	SEI;
	PCD = RM16( irq_vector );
	CHANGE_PC();
}

/* check OCI or TOI */
static void check_timer_event(void)
{
	/* OCI */
	if( CTD >= OCD)
	{
		OCH++;	// next IRQ point
		m6800.tcsr |= TCSR_OCF;
		m6800.pending_tcsr |= TCSR_OCF;
		MODIFIED_tcsr;
		if ( !(CC & 0x10) && (m6800.tcsr & TCSR_EOCI))
			TAKE_OCI;
	}
	/* TOI */
	if( CTD >= TOD)
	{
		TOH++;	// next IRQ point
#if 0
		CLEANUP_conters;
#endif
		m6800.tcsr |= TCSR_TOF;
		m6800.pending_tcsr |= TCSR_TOF;
		MODIFIED_tcsr;
		if ( !(CC & 0x10) && (m6800.tcsr & TCSR_ETOI))
			TAKE_TOI;
	}
	/* set next event */
	SET_TIMER_EVENT;
}

#ifdef __cplusplus
extern "C" {
#endif

M6800_INLINE void aba(void);
M6800_INLINE void abx(void);
M6800_INLINE void adca_di(void);
M6800_INLINE void adca_ex(void);
M6800_INLINE void adca_im(void);
M6800_INLINE void adca_ix(void);
M6800_INLINE void adcb_di(void);
M6800_INLINE void adcb_ex(void);
M6800_INLINE void adcb_im(void);
M6800_INLINE void adcb_ix(void);
//M6800_INLINE void adcx_im(void);
M6800_INLINE void adda_di(void);
M6800_INLINE void adda_ex(void);
M6800_INLINE void adda_im(void);
M6800_INLINE void adda_ix(void);
M6800_INLINE void addb_di(void);
M6800_INLINE void addb_ex(void);
M6800_INLINE void addb_im(void);
M6800_INLINE void addb_ix(void);
M6800_INLINE void addd_di(void);
M6800_INLINE void addd_ex(void);
M6800_INLINE void addx_ex(void);
M6800_INLINE void addd_im(void);
M6800_INLINE void addd_ix(void);
M6800_INLINE void aim_di(void);
M6800_INLINE void aim_ix(void);
M6800_INLINE void anda_di(void);
M6800_INLINE void anda_ex(void);
M6800_INLINE void anda_im(void);
M6800_INLINE void anda_ix(void);
M6800_INLINE void andb_di(void);
M6800_INLINE void andb_ex(void);
M6800_INLINE void andb_im(void);
M6800_INLINE void andb_ix(void);
M6800_INLINE void asl_ex(void);
M6800_INLINE void asl_ix(void);
M6800_INLINE void asla(void);
M6800_INLINE void aslb(void);
M6800_INLINE void asld(void);
M6800_INLINE void asr_ex(void);
M6800_INLINE void asr_ix(void);
M6800_INLINE void asra(void);
M6800_INLINE void asrb(void);
M6800_INLINE void bcc(void);
M6800_INLINE void bcs(void);
M6800_INLINE void beq(void);
M6800_INLINE void bge(void);
M6800_INLINE void bgt(void);
M6800_INLINE void bhi(void);
M6800_INLINE void bita_di(void);
M6800_INLINE void bita_ex(void);
M6800_INLINE void bita_im(void);
M6800_INLINE void bita_ix(void);
M6800_INLINE void bitb_di(void);
M6800_INLINE void bitb_ex(void);
M6800_INLINE void bitb_im(void);
M6800_INLINE void bitb_ix(void);
M6800_INLINE void ble(void);
M6800_INLINE void bls(void);
M6800_INLINE void blt(void);
M6800_INLINE void bmi(void);
M6800_INLINE void bne(void);
M6800_INLINE void bpl(void);
M6800_INLINE void bra(void);
M6800_INLINE void brn(void);
M6800_INLINE void bsr(void);
M6800_INLINE void bvc(void);
M6800_INLINE void bvs(void);
M6800_INLINE void cba(void);
M6800_INLINE void clc(void);
M6800_INLINE void cli(void);
M6800_INLINE void clr_ex(void);
M6800_INLINE void clr_ix(void);
M6800_INLINE void clra(void);
M6800_INLINE void clrb(void);
M6800_INLINE void clv(void);
M6800_INLINE void cmpa_di(void);
M6800_INLINE void cmpa_ex(void);
M6800_INLINE void cmpa_im(void);
M6800_INLINE void cmpa_ix(void);
M6800_INLINE void cmpb_di(void);
M6800_INLINE void cmpb_ex(void);
M6800_INLINE void cmpb_im(void);
M6800_INLINE void cmpb_ix(void);
M6800_INLINE void cmpx_di(void);
M6800_INLINE void cmpx_ex(void);
M6800_INLINE void cmpx_im(void);
M6800_INLINE void cmpx_ix(void);
M6800_INLINE void com_ex(void);
M6800_INLINE void com_ix(void);
M6800_INLINE void coma(void);
M6800_INLINE void comb(void);
M6800_INLINE void daa(void);
M6800_INLINE void dec_ex(void);
M6800_INLINE void dec_ix(void);
M6800_INLINE void deca(void);
M6800_INLINE void decb(void);
M6800_INLINE void des(void);
M6800_INLINE void dex(void);
M6800_INLINE void eim_di(void);
M6800_INLINE void eim_ix(void);
M6800_INLINE void eora_di(void);
M6800_INLINE void eora_ex(void);
M6800_INLINE void eora_im(void);
M6800_INLINE void eora_ix(void);
M6800_INLINE void eorb_di(void);
M6800_INLINE void eorb_ex(void);
M6800_INLINE void eorb_im(void);
M6800_INLINE void eorb_ix(void);
//M6800_INLINE void illegal(void);
static void illegal(void);
M6800_INLINE void inc_ex(void);
M6800_INLINE void inc_ix(void);
M6800_INLINE void inca(void);
M6800_INLINE void incb(void);
M6800_INLINE void ins(void);
M6800_INLINE void inx(void);
M6800_INLINE void jmp_ex(void);
M6800_INLINE void jmp_ix(void);
M6800_INLINE void jsr_di(void);
M6800_INLINE void jsr_ex(void);
M6800_INLINE void jsr_ix(void);
M6800_INLINE void lda_di(void);
M6800_INLINE void lda_ex(void);
M6800_INLINE void lda_im(void);
M6800_INLINE void lda_ix(void);
M6800_INLINE void ldb_di(void);
M6800_INLINE void ldb_ex(void);
M6800_INLINE void ldb_im(void);
M6800_INLINE void ldb_ix(void);
M6800_INLINE void ldd_di(void);
M6800_INLINE void ldd_ex(void);
M6800_INLINE void ldd_im(void);
M6800_INLINE void ldd_ix(void);
M6800_INLINE void lds_di(void);
M6800_INLINE void lds_ex(void);
M6800_INLINE void lds_im(void);
M6800_INLINE void lds_ix(void);
M6800_INLINE void ldx_di(void);
M6800_INLINE void ldx_ex(void);
M6800_INLINE void ldx_im(void);
M6800_INLINE void ldx_ix(void);
M6800_INLINE void lsr_ex(void);
M6800_INLINE void lsr_ix(void);
M6800_INLINE void lsra(void);
M6800_INLINE void lsrb(void);
M6800_INLINE void lsrd(void);
M6800_INLINE void mul(void);
M6800_INLINE void neg_ex(void);
M6800_INLINE void neg_ix(void);
M6800_INLINE void nega(void);
M6800_INLINE void negb(void);
M6800_INLINE void nop(void);
M6800_INLINE void oim_di(void);
M6800_INLINE void oim_ix(void);
M6800_INLINE void ora_di(void);
M6800_INLINE void ora_ex(void);
M6800_INLINE void ora_im(void);
M6800_INLINE void ora_ix(void);
M6800_INLINE void orb_di(void);
M6800_INLINE void orb_ex(void);
M6800_INLINE void orb_im(void);
M6800_INLINE void orb_ix(void);
M6800_INLINE void psha(void);
M6800_INLINE void pshb(void);
M6800_INLINE void pshx(void);
M6800_INLINE void pula(void);
M6800_INLINE void pulb(void);
M6800_INLINE void pulx(void);
M6800_INLINE void rol_ex(void);
M6800_INLINE void rol_ix(void);
M6800_INLINE void rola(void);
M6800_INLINE void rolb(void);
M6800_INLINE void ror_ex(void);
M6800_INLINE void ror_ix(void);
M6800_INLINE void rora(void);
M6800_INLINE void rorb(void);
M6800_INLINE void rti(void);
M6800_INLINE void rts(void);
M6800_INLINE void sba(void);
M6800_INLINE void sbca_di(void);
M6800_INLINE void sbca_ex(void);
M6800_INLINE void sbca_im(void);
M6800_INLINE void sbca_ix(void);
M6800_INLINE void sbcb_di(void);
M6800_INLINE void sbcb_ex(void);
M6800_INLINE void sbcb_im(void);
M6800_INLINE void sbcb_ix(void);
M6800_INLINE void sec(void);
M6800_INLINE void sei(void);
M6800_INLINE void sev(void);
#if (HAS_HD63701)
M6800_INLINE void slp(void);
#endif
M6800_INLINE void sta_di(void);
M6800_INLINE void sta_ex(void);
M6800_INLINE void sta_im(void);
M6800_INLINE void sta_ix(void);
M6800_INLINE void stb_di(void);
M6800_INLINE void stb_ex(void);
M6800_INLINE void stb_im(void);
M6800_INLINE void stb_ix(void);
M6800_INLINE void std_di(void);
M6800_INLINE void std_ex(void);
M6800_INLINE void std_im(void);
M6800_INLINE void std_ix(void);
M6800_INLINE void sts_di(void);
M6800_INLINE void sts_ex(void);
M6800_INLINE void sts_im(void);
M6800_INLINE void sts_ix(void);
M6800_INLINE void stx_di(void);
M6800_INLINE void stx_ex(void);
M6800_INLINE void stx_im(void);
M6800_INLINE void stx_ix(void);
M6800_INLINE void suba_di(void);
M6800_INLINE void suba_ex(void);
M6800_INLINE void suba_im(void);
M6800_INLINE void suba_ix(void);
M6800_INLINE void subb_di(void);
M6800_INLINE void subb_ex(void);
M6800_INLINE void subb_im(void);
M6800_INLINE void subb_ix(void);
M6800_INLINE void subd_di(void);
M6800_INLINE void subd_ex(void);
M6800_INLINE void subd_im(void);
M6800_INLINE void subd_ix(void);
M6800_INLINE void swi(void);
M6800_INLINE void tab(void);
M6800_INLINE void tap(void);
M6800_INLINE void tba(void);
M6800_INLINE void tim_di(void);
M6800_INLINE void tim_ix(void);
M6800_INLINE void tpa(void);
M6800_INLINE void tst_ex(void);
M6800_INLINE void tst_ix(void);
M6800_INLINE void tsta(void);
M6800_INLINE void tstb(void);
M6800_INLINE void tsx(void);
M6800_INLINE void txs(void);
M6800_INLINE void undoc1(void);
M6800_INLINE void undoc2(void);
M6800_INLINE void wai(void);
M6800_INLINE void xgdx(void);

M6800_INLINE void cpx_di(void);
M6800_INLINE void cpx_ex(void);
M6800_INLINE void cpx_im(void);
M6800_INLINE void cpx_ix(void);
#if (HAS_HD63701)
//M6800_INLINE void trap(void);
static void trap(void);
#endif

static void (*m6800_insn[0x100])(void) = {
illegal,nop,	illegal,illegal,illegal,illegal,tap,	tpa,
inx,	dex,	clv,	sev,	clc,	sec,	cli,	sei,
sba,	cba,	illegal,illegal,illegal,illegal,tab,	tba,
illegal,daa,	illegal,aba,	illegal,illegal,illegal,illegal,
bra,	brn,	bhi,	bls,	bcc,	bcs,	bne,	beq,
bvc,	bvs,	bpl,	bmi,	bge,	blt,	bgt,	ble,
tsx,	ins,	pula,	pulb,	des,	txs,	psha,	pshb,
illegal,rts,	illegal,rti,	illegal,illegal,wai,	swi,
nega,	illegal,illegal,coma,	lsra,	illegal,rora,	asra,
asla,	rola,	deca,	illegal,inca,	tsta,	illegal,clra,
negb,	illegal,illegal,comb,	lsrb,	illegal,rorb,	asrb,
aslb,	rolb,	decb,	illegal,incb,	tstb,	illegal,clrb,
neg_ix, illegal,illegal,com_ix, lsr_ix, illegal,ror_ix, asr_ix,
asl_ix, rol_ix, dec_ix, illegal,inc_ix, tst_ix, jmp_ix, clr_ix,
neg_ex, illegal,illegal,com_ex, lsr_ex, illegal,ror_ex, asr_ex,
asl_ex, rol_ex, dec_ex, illegal,inc_ex, tst_ex, jmp_ex, clr_ex,
suba_im,cmpa_im,sbca_im,illegal,anda_im,bita_im,lda_im, sta_im,
eora_im,adca_im,ora_im, adda_im,cmpx_im,bsr,	lds_im, sts_im,
suba_di,cmpa_di,sbca_di,illegal,anda_di,bita_di,lda_di, sta_di,
eora_di,adca_di,ora_di, adda_di,cmpx_di,jsr_di, lds_di, sts_di,
suba_ix,cmpa_ix,sbca_ix,illegal,anda_ix,bita_ix,lda_ix, sta_ix,
eora_ix,adca_ix,ora_ix, adda_ix,cmpx_ix,jsr_ix, lds_ix, sts_ix,
suba_ex,cmpa_ex,sbca_ex,illegal,anda_ex,bita_ex,lda_ex, sta_ex,
eora_ex,adca_ex,ora_ex, adda_ex,cmpx_ex,jsr_ex, lds_ex, sts_ex,
subb_im,cmpb_im,sbcb_im,illegal,andb_im,bitb_im,ldb_im, stb_im,
eorb_im,adcb_im,orb_im, addb_im,illegal,illegal,ldx_im, stx_im,
subb_di,cmpb_di,sbcb_di,illegal,andb_di,bitb_di,ldb_di, stb_di,
eorb_di,adcb_di,orb_di, addb_di,illegal,illegal,ldx_di, stx_di,
subb_ix,cmpb_ix,sbcb_ix,illegal,andb_ix,bitb_ix,ldb_ix, stb_ix,
eorb_ix,adcb_ix,orb_ix, addb_ix,illegal,illegal,ldx_ix, stx_ix,
subb_ex,cmpb_ex,sbcb_ex,illegal,andb_ex,bitb_ex,ldb_ex, stb_ex,
eorb_ex,adcb_ex,orb_ex, addb_ex,illegal,illegal,ldx_ex, stx_ex
};

#if (HAS_M6801||HAS_M6803)
static void (*m6803_insn[0x100])(void) = {
illegal,nop,	illegal,illegal,lsrd,	asld,	tap,	tpa,
inx,	dex,	clv,	sev,	clc,	sec,	cli,	sei,
sba,	cba,	illegal,illegal,illegal,illegal,tab,	tba,
illegal,daa,	illegal,aba,	illegal,illegal,illegal,illegal,
bra,	brn,	bhi,	bls,	bcc,	bcs,	bne,	beq,
bvc,	bvs,	bpl,	bmi,	bge,	blt,	bgt,	ble,
tsx,	ins,	pula,	pulb,	des,	txs,	psha,	pshb,
pulx,	rts,	abx,	rti,	pshx,	mul,	wai,	swi,
nega,	illegal,illegal,coma,	lsra,	illegal,rora,	asra,
asla,	rola,	deca,	illegal,inca,	tsta,	illegal,clra,
negb,	illegal,illegal,comb,	lsrb,	illegal,rorb,	asrb,
aslb,	rolb,	decb,	illegal,incb,	tstb,	illegal,clrb,
neg_ix, illegal,illegal,com_ix, lsr_ix, illegal,ror_ix, asr_ix,
asl_ix, rol_ix, dec_ix, illegal,inc_ix, tst_ix, jmp_ix, clr_ix,
neg_ex, illegal,illegal,com_ex, lsr_ex, illegal,ror_ex, asr_ex,
asl_ex, rol_ex, dec_ex, illegal,inc_ex, tst_ex, jmp_ex, clr_ex,
suba_im,cmpa_im,sbca_im,subd_im,anda_im,bita_im,lda_im, sta_im,
eora_im,adca_im,ora_im, adda_im,cpx_im ,bsr,	lds_im, sts_im,
suba_di,cmpa_di,sbca_di,subd_di,anda_di,bita_di,lda_di, sta_di,
eora_di,adca_di,ora_di, adda_di,cpx_di ,jsr_di, lds_di, sts_di,
suba_ix,cmpa_ix,sbca_ix,subd_ix,anda_ix,bita_ix,lda_ix, sta_ix,
eora_ix,adca_ix,ora_ix, adda_ix,cpx_ix ,jsr_ix, lds_ix, sts_ix,
suba_ex,cmpa_ex,sbca_ex,subd_ex,anda_ex,bita_ex,lda_ex, sta_ex,
eora_ex,adca_ex,ora_ex, adda_ex,cpx_ex ,jsr_ex, lds_ex, sts_ex,
subb_im,cmpb_im,sbcb_im,addd_im,andb_im,bitb_im,ldb_im, stb_im,
eorb_im,adcb_im,orb_im, addb_im,ldd_im, std_im, ldx_im, stx_im,
subb_di,cmpb_di,sbcb_di,addd_di,andb_di,bitb_di,ldb_di, stb_di,
eorb_di,adcb_di,orb_di, addb_di,ldd_di, std_di, ldx_di, stx_di,
subb_ix,cmpb_ix,sbcb_ix,addd_ix,andb_ix,bitb_ix,ldb_ix, stb_ix,
eorb_ix,adcb_ix,orb_ix, addb_ix,ldd_ix, std_ix, ldx_ix, stx_ix,
subb_ex,cmpb_ex,sbcb_ex,addd_ex,andb_ex,bitb_ex,ldb_ex, stb_ex,
eorb_ex,adcb_ex,orb_ex, addb_ex,ldd_ex, std_ex, ldx_ex, stx_ex
};
#endif

#if (HAS_HD63701)
static void (*hd63701_insn[0x100])(void) = {
trap	,nop,	trap	,trap	,lsrd,	asld,	tap,	tpa,
inx,	dex,	clv,	sev,	clc,	sec,	cli,	sei,
sba,	cba,	undoc1, undoc2, trap	,trap	,tab,	tba,
xgdx,	daa,	slp		,aba,	trap	,trap	,trap	,trap	,
bra,	brn,	bhi,	bls,	bcc,	bcs,	bne,	beq,
bvc,	bvs,	bpl,	bmi,	bge,	blt,	bgt,	ble,
tsx,	ins,	pula,	pulb,	des,	txs,	psha,	pshb,
pulx,	rts,	abx,	rti,	pshx,	mul,	wai,	swi,
nega,	trap	,trap	,coma,	lsra,	trap	,rora,	asra,
asla,	rola,	deca,	trap	,inca,	tsta,	trap	,clra,
negb,	trap	,trap	,comb,	lsrb,	trap	,rorb,	asrb,
aslb,	rolb,	decb,	trap	,incb,	tstb,	trap	,clrb,
neg_ix, aim_ix, oim_ix, com_ix, lsr_ix, eim_ix, ror_ix, asr_ix,
asl_ix, rol_ix, dec_ix, tim_ix, inc_ix, tst_ix, jmp_ix, clr_ix,
neg_ex, aim_di, oim_di, com_ex, lsr_ex, eim_di, ror_ex, asr_ex,
asl_ex, rol_ex, dec_ex, tim_di, inc_ex, tst_ex, jmp_ex, clr_ex,
suba_im,cmpa_im,sbca_im,subd_im,anda_im,bita_im,lda_im, sta_im,
eora_im,adca_im,ora_im, adda_im,cpx_im ,bsr,	lds_im, sts_im,
suba_di,cmpa_di,sbca_di,subd_di,anda_di,bita_di,lda_di, sta_di,
eora_di,adca_di,ora_di, adda_di,cpx_di ,jsr_di, lds_di, sts_di,
suba_ix,cmpa_ix,sbca_ix,subd_ix,anda_ix,bita_ix,lda_ix, sta_ix,
eora_ix,adca_ix,ora_ix, adda_ix,cpx_ix ,jsr_ix, lds_ix, sts_ix,
suba_ex,cmpa_ex,sbca_ex,subd_ex,anda_ex,bita_ex,lda_ex, sta_ex,
eora_ex,adca_ex,ora_ex, adda_ex,cpx_ex ,jsr_ex, lds_ex, sts_ex,
subb_im,cmpb_im,sbcb_im,addd_im,andb_im,bitb_im,ldb_im, stb_im,
eorb_im,adcb_im,orb_im, addb_im,ldd_im, std_im, ldx_im, stx_im,
subb_di,cmpb_di,sbcb_di,addd_di,andb_di,bitb_di,ldb_di, stb_di,
eorb_di,adcb_di,orb_di, addb_di,ldd_di, std_di, ldx_di, stx_di,
subb_ix,cmpb_ix,sbcb_ix,addd_ix,andb_ix,bitb_ix,ldb_ix, stb_ix,
eorb_ix,adcb_ix,orb_ix, addb_ix,ldd_ix, std_ix, ldx_ix, stx_ix,
subb_ex,cmpb_ex,sbcb_ex,addd_ex,andb_ex,bitb_ex,ldb_ex, stb_ex,
eorb_ex,adcb_ex,orb_ex, addb_ex,ldd_ex, std_ex, ldx_ex, stx_ex
};
#endif

#if (HAS_NSC8105)
static void (*nsc8105_insn[0x100])(void) = {
illegal,illegal,nop,	illegal,illegal,tap,	illegal,tpa,
inx,	clv,	dex,	sev,	clc,	cli,	sec,	sei,
sba,	illegal,cba,	illegal,illegal,tab,	illegal,tba,
illegal,illegal,daa,	aba,	illegal,illegal,illegal,illegal,
bra,	bhi,	brn,	bls,	bcc,	bne,	bcs,	beq,
bvc,	bpl,	bvs,	bmi,	bge,	bgt,	blt,	ble,
tsx,	pula,	ins,	pulb,	des,	psha,	txs,	pshb,
illegal,illegal,rts,	rti,	illegal,wai,	illegal,swi,
suba_im,sbca_im,cmpa_im,illegal,anda_im,lda_im, bita_im,sta_im,
eora_im,ora_im, adca_im,adda_im,cmpx_im,lds_im, bsr,	sts_im,
suba_di,sbca_di,cmpa_di,illegal,anda_di,lda_di, bita_di,sta_di,
eora_di,ora_di, adca_di,adda_di,cmpx_di,lds_di, jsr_di, sts_di,
suba_ix,sbca_ix,cmpa_ix,illegal,anda_ix,lda_ix, bita_ix,sta_ix,
eora_ix,ora_ix, adca_ix,adda_ix,cmpx_ix,lds_ix, jsr_ix, sts_ix,
suba_ex,sbca_ex,cmpa_ex,illegal,anda_ex,lda_ex, bita_ex,sta_ex,
eora_ex,ora_ex, adca_ex,adda_ex,cmpx_ex,lds_ex, jsr_ex, sts_ex,
nega,	illegal,illegal,coma,	lsra,	rora,	illegal,asra,
asla,	deca,	rola,	illegal,inca,	illegal,tsta,	clra,
negb,	illegal,illegal,comb,	lsrb,	rorb,	illegal,asrb,
aslb,	decb,	rolb,	illegal,incb,	illegal,tstb,	clrb,
neg_ix, illegal,illegal,com_ix, lsr_ix, ror_ix,	illegal,asr_ix,
asl_ix, dec_ix, rol_ix, illegal,inc_ix, jmp_ix, tst_ix, clr_ix,
neg_ex, illegal,illegal,com_ex, lsr_ex, ror_ex,	illegal,asr_ex,
asl_ex, dec_ex, rol_ex, illegal,inc_ex, jmp_ex, tst_ex, clr_ex,
subb_im,sbcb_im,cmpb_im,illegal,andb_im,ldb_im, bitb_im,stb_im,
eorb_im,orb_im, adcb_im,addb_im,illegal,ldx_im, illegal,stx_im,
subb_di,sbcb_di,cmpb_di,illegal,andb_di,ldb_di, bitb_di,stb_di,
eorb_di,orb_di, adcb_di,addb_di,illegal,ldx_di, illegal,stx_di,
subb_ix,sbcb_ix,cmpb_ix,illegal,andb_ix,ldb_ix, bitb_ix,stb_ix,
eorb_ix,orb_ix, adcb_ix,addb_ix,adcx_im,ldx_ix, illegal,stx_ix,
subb_ex,sbcb_ex,cmpb_ex,illegal,andb_ex,ldb_ex, bitb_ex,stb_ex,
eorb_ex,orb_ex, adcb_ex,addb_ex,addx_ex,ldx_ex, illegal,stx_ex
};
#endif


/*

HNZVC

? = undefined
* = affected
- = unaffected
0 = cleared
1 = set
# = ccr directly affected by instruction
@ = special - carry set if bit 7 is set

*/

//M6800_INLINE void illegal( void )
static void illegal( void )
{
//	logerror("M6808: illegal opcode: address %04X, op %02X\n",PC,(int) M_RDOP_ARG(PC)&0xFF);
}

/* HD63701 only */
#if (HAS_HD63701)
//M6800_INLINE void trap( void )
static void trap( void )
{
//	logerror("M6808: illegal opcode: address %04X, op %02X\n",PC,(int) M_RDOP_ARG(PC)&0xFF);
	TAKE_TRAP;
}
#endif

/* $00 ILLEGAL */

/* $01 NOP */
M6800_INLINE void nop( void )
{
}

/* $02 ILLEGAL */

/* $03 ILLEGAL */

/* $04 LSRD inherent -0*-* */
M6800_INLINE void lsrd (void)
{
	UINT16 t;
	CLR_NZC; t = D; CC|=(t&0x0001);
	t>>=1; SET_Z16(t); D=t;
}

/* $05 ASLD inherent ?**** */
M6800_INLINE void asld (void)
{
	int r;
	UINT16 t;
	t = D; r=t<<1;
	CLR_NZVC; SET_FLAGS16(t,t,r);
	D=r;
}

/* $06 TAP inherent ##### */
M6800_INLINE void tap (void)
{
	CC=A;
	ONE_MORE_INSN();
	CHECK_IRQ_LINES(); /* HJB 990417 */
}

/* $07 TPA inherent ----- */
M6800_INLINE void tpa (void)
{
	A=CC;
}

/* $08 INX inherent --*-- */
M6800_INLINE void inx (void)
{
	++X;
	CLR_Z; SET_Z16(X);
}

/* $09 DEX inherent --*-- */
M6800_INLINE void dex (void)
{
	--X;
	CLR_Z; SET_Z16(X);
}

/* $0a CLV */
M6800_INLINE void clv (void)
{
	CLV;
}

/* $0b SEV */
M6800_INLINE void sev (void)
{
	SEV;
}

/* $0c CLC */
M6800_INLINE void clc (void)
{
	CLC;
}

/* $0d SEC */
M6800_INLINE void sec (void)
{
	SEC;
}

/* $0e CLI */
M6800_INLINE void cli (void)
{
	CLI;
	ONE_MORE_INSN();
	CHECK_IRQ_LINES(); /* HJB 990417 */
}

/* $0f SEI */
M6800_INLINE void sei (void)
{
	SEI;
	ONE_MORE_INSN();
	CHECK_IRQ_LINES(); /* HJB 990417 */
}

/* $10 SBA inherent -**** */
M6800_INLINE void sba (void)
{
	UINT16 t;
	t=A-B;
	CLR_NZVC; SET_FLAGS8(A,B,t);
	A=t;
}

/* $11 CBA inherent -**** */
M6800_INLINE void cba (void)
{
	UINT16 t;
	t=A-B;
	CLR_NZVC; SET_FLAGS8(A,B,t);
}

/* $12 ILLEGAL */
M6800_INLINE void undoc1 (void)
{
	X += RM( S + 1 );
}

/* $13 ILLEGAL */
M6800_INLINE void undoc2 (void)
{
	X += RM( S + 1 );
}


/* $14 ILLEGAL */

/* $15 ILLEGAL */

/* $16 TAB inherent -**0- */
M6800_INLINE void tab (void)
{
	B=A;
	CLR_NZV; SET_NZ8(B);
}

/* $17 TBA inherent -**0- */
M6800_INLINE void tba (void)
{
	A=B;
	CLR_NZV; SET_NZ8(A);
}

/* $18 XGDX inherent ----- */ /* HD63701YO only */
M6800_INLINE void xgdx( void )
{
	UINT16 t = X;
	X = D;
	D=t;
}

/* $19 DAA inherent (A) -**0* */
M6800_INLINE void daa( void )
{
	UINT8 msn, lsn;
	UINT16 t, cf = 0;
	msn=A & 0xf0; lsn=A & 0x0f;
	if( lsn>0x09 || CC&0x20 ) cf |= 0x06;
	if( msn>0x80 && lsn>0x09 ) cf |= 0x60;
	if( msn>0x90 || CC&0x01 ) cf |= 0x60;
	t = cf + A;
	CLR_NZV; /* keep carry from previous operation */
	SET_NZ8((UINT8)t); SET_C8(t);
	A = t;
}

/* $1a ILLEGAL */

#if (HAS_HD63701)
/* $1a SLP */ /* HD63701YO only */
M6800_INLINE void slp (void)
{
	/* wait for next IRQ (same as waiting of wai) */
	m6808.wai_state |= HD63701_SLP;
	EAT_CYCLES;
}
#endif

/* $1b ABA inherent ***** */
M6800_INLINE void aba (void)
{
	UINT16 t;
	t=A+B;
	CLR_HNZVC; SET_FLAGS8(A,B,t); SET_H(A,B,t);
	A=t;
}

/* $1c ILLEGAL */

/* $1d ILLEGAL */

/* $1e ILLEGAL */

/* $1f ILLEGAL */

/* $20 BRA relative ----- */
M6800_INLINE void bra( void )
{
	UINT8 t;
	IMMBYTE(t);PC+=SIGNED(t);CHANGE_PC();
	/* speed up busy loops */
	if (t==0xfe) EAT_CYCLES;
}

/* $21 BRN relative ----- */
M6800_INLINE void brn( void )
{
	UINT8 t;
	IMMBYTE(t);
}

/* $22 BHI relative ----- */
M6800_INLINE void bhi( void )
{
	UINT8 t;
	BRANCH(!(CC&0x05));
}

/* $23 BLS relative ----- */
M6800_INLINE void bls( void )
{
	UINT8 t;
	BRANCH(CC&0x05);
}

/* $24 BCC relative ----- */
M6800_INLINE void bcc( void )
{
	UINT8 t;
	BRANCH(!(CC&0x01));
}

/* $25 BCS relative ----- */
M6800_INLINE void bcs( void )
{
	UINT8 t;
	BRANCH(CC&0x01);
}

/* $26 BNE relative ----- */
M6800_INLINE void bne( void )
{
	UINT8 t;
	BRANCH(!(CC&0x04));
}

/* $27 BEQ relative ----- */
M6800_INLINE void beq( void )
{
	UINT8 t;
	BRANCH(CC&0x04);
}

/* $28 BVC relative ----- */
M6800_INLINE void bvc( void )
{
	UINT8 t;
	BRANCH(!(CC&0x02));
}

/* $29 BVS relative ----- */
M6800_INLINE void bvs( void )
{
	UINT8 t;
	BRANCH(CC&0x02);
}

/* $2a BPL relative ----- */
M6800_INLINE void bpl( void )
{
	UINT8 t;
	BRANCH(!(CC&0x08));
}

/* $2b BMI relative ----- */
M6800_INLINE void bmi( void )
{
	UINT8 t;
	BRANCH(CC&0x08);
}

/* $2c BGE relative ----- */
M6800_INLINE void bge( void )
{
	UINT8 t;
	BRANCH(!NXORV);
}

/* $2d BLT relative ----- */
M6800_INLINE void blt( void )
{
	UINT8 t;
	BRANCH(NXORV);
}

/* $2e BGT relative ----- */
M6800_INLINE void bgt( void )
{
	UINT8 t;
	BRANCH(!(NXORV||CC&0x04));
}

/* $2f BLE relative ----- */
M6800_INLINE void ble( void )
{
	UINT8 t;
	BRANCH(NXORV||CC&0x04);
}

/* $30 TSX inherent ----- */
M6800_INLINE void tsx (void)
{
	X = ( S + 1 );
}

/* $31 INS inherent ----- */
M6800_INLINE void ins (void)
{
	++S;
}

/* $32 PULA inherent ----- */
M6800_INLINE void pula (void)
{
	PULLBYTE(m6808.d.b.h);
}

/* $33 PULB inherent ----- */
M6800_INLINE void pulb (void)
{
	PULLBYTE(m6808.d.b.l);
}

/* $34 DES inherent ----- */
M6800_INLINE void des (void)
{
	--S;
}

/* $35 TXS inherent ----- */
M6800_INLINE void txs (void)
{
	S = ( X - 1 );
}

/* $36 PSHA inherent ----- */
M6800_INLINE void psha (void)
{
	PUSHBYTE(m6808.d.b.h);
}

/* $37 PSHB inherent ----- */
M6800_INLINE void pshb (void)
{
	PUSHBYTE(m6808.d.b.l);
}

/* $38 PULX inherent ----- */
M6800_INLINE void pulx (void)
{
	PULLWORD(pX);
}

/* $39 RTS inherent ----- */
M6800_INLINE void rts( void )
{
	PULLWORD(pPC);
	CHANGE_PC();
}

/* $3a ABX inherent ----- */
M6800_INLINE void abx( void )
{
	X += B;
}

/* $3b RTI inherent ##### */
M6800_INLINE void rti( void )
{
	PULLBYTE(CC);
	PULLBYTE(B);
	PULLBYTE(A);
	PULLWORD(pX);
	PULLWORD(pPC);
	CHANGE_PC();
	CHECK_IRQ_LINES(); /* HJB 990417 */
}

/* $3c PSHX inherent ----- */
M6800_INLINE void pshx (void)
{
	PUSHWORD(pX);
}

/* $3d MUL inherent --*-@ */
M6800_INLINE void mul( void )
{
	UINT16 t;
	t=A*B;
	CLR_C; if(t&0x80) SEC;
	D=t;
}

/* $3e WAI inherent ----- */
M6800_INLINE void wai( void )
{
	/*
     * WAI stacks the entire machine state on the
     * hardware stack, then waits for an interrupt.
     */
	m6808.wai_state |= M6800_WAI;
	PUSHWORD(pPC);
	PUSHWORD(pX);
	PUSHBYTE(A);
	PUSHBYTE(B);
	PUSHBYTE(CC);
	CHECK_IRQ_LINES();
	if (m6808.wai_state & M6800_WAI) EAT_CYCLES;
}

/* $3f SWI absolute indirect ----- */
M6800_INLINE void swi( void )
{
	PUSHWORD(pPC);
	PUSHWORD(pX);
	PUSHBYTE(A);
	PUSHBYTE(B);
    PUSHBYTE(CC);
    SEI;
	PCD = RM16(0xfffa);
	CHANGE_PC();
}

/* $40 NEGA inherent ?**** */
M6800_INLINE void nega( void )
{
	UINT16 r;
	r=-A;
	CLR_NZVC; SET_FLAGS8(0,A,r);
	A=r;
}

/* $41 ILLEGAL */

/* $42 ILLEGAL */

/* $43 COMA inherent -**01 */
M6800_INLINE void coma( void )
{
	A = ~A;
	CLR_NZV; SET_NZ8(A); SEC;
}

/* $44 LSRA inherent -0*-* */
M6800_INLINE void lsra( void )
{
	CLR_NZC; CC|=(A&0x01);
	A>>=1; SET_Z8(A);
}

/* $45 ILLEGAL */

/* $46 RORA inherent -**-* */
M6800_INLINE void rora( void )
{
	UINT8 r;
	r=(CC&0x01)<<7;
	CLR_NZC; CC|=(A&0x01);
	r |= A>>1; SET_NZ8(r);
	A=r;
}

/* $47 ASRA inherent ?**-* */
M6800_INLINE void asra( void )
{
	CLR_NZC; CC|=(A&0x01);
	A>>=1; A|=((A&0x40)<<1);
	SET_NZ8(A);
}

/* $48 ASLA inherent ?**** */
M6800_INLINE void asla( void )
{
	UINT16 r;
	r=A<<1;
	CLR_NZVC; SET_FLAGS8(A,A,r);
	A=r;
}

/* $49 ROLA inherent -**** */
M6800_INLINE void rola( void )
{
	UINT16 t,r;
	t = A; r = CC&0x01; r |= t<<1;
	CLR_NZVC; SET_FLAGS8(t,t,r);
	A=r;
}

/* $4a DECA inherent -***- */
M6800_INLINE void deca( void )
{
	--A;
	CLR_NZV; SET_FLAGS8D(A);
}

/* $4b ILLEGAL */

/* $4c INCA inherent -***- */
M6800_INLINE void inca( void )
{
	++A;
	CLR_NZV; SET_FLAGS8I(A);
}

/* $4d TSTA inherent -**0- */
M6800_INLINE void tsta( void )
{
	CLR_NZVC; SET_NZ8(A);
}

/* $4e ILLEGAL */

/* $4f CLRA inherent -0100 */
M6800_INLINE void clra( void )
{
	A=0;
	CLR_NZVC; SEZ;
}

/* $50 NEGB inherent ?**** */
M6800_INLINE void negb( void )
{
	UINT16 r;
	r=-B;
	CLR_NZVC; SET_FLAGS8(0,B,r);
	B=r;
}

/* $51 ILLEGAL */

/* $52 ILLEGAL */

/* $53 COMB inherent -**01 */
M6800_INLINE void comb( void )
{
	B = ~B;
	CLR_NZV; SET_NZ8(B); SEC;
}

/* $54 LSRB inherent -0*-* */
M6800_INLINE void lsrb( void )
{
	CLR_NZC; CC|=(B&0x01);
	B>>=1; SET_Z8(B);
}

/* $55 ILLEGAL */

/* $56 RORB inherent -**-* */
M6800_INLINE void rorb( void )
{
	UINT8 r;
	r=(CC&0x01)<<7;
	CLR_NZC; CC|=(B&0x01);
	r |= B>>1; SET_NZ8(r);
	B=r;
}

/* $57 ASRB inherent ?**-* */
M6800_INLINE void asrb( void )
{
	CLR_NZC; CC|=(B&0x01);
	B>>=1; B|=((B&0x40)<<1);
	SET_NZ8(B);
}

/* $58 ASLB inherent ?**** */
M6800_INLINE void aslb( void )
{
	UINT16 r;
	r=B<<1;
	CLR_NZVC; SET_FLAGS8(B,B,r);
	B=r;
}

/* $59 ROLB inherent -**** */
M6800_INLINE void rolb( void )
{
	UINT16 t,r;
	t = B; r = CC&0x01; r |= t<<1;
	CLR_NZVC; SET_FLAGS8(t,t,r);
	B=r;
}

/* $5a DECB inherent -***- */
M6800_INLINE void decb( void )
{
	--B;
	CLR_NZV; SET_FLAGS8D(B);
}

/* $5b ILLEGAL */

/* $5c INCB inherent -***- */
M6800_INLINE void incb( void )
{
	++B;
	CLR_NZV; SET_FLAGS8I(B);
}

/* $5d TSTB inherent -**0- */
M6800_INLINE void tstb( void )
{
	CLR_NZVC; SET_NZ8(B);
}

/* $5e ILLEGAL */

/* $5f CLRB inherent -0100 */
M6800_INLINE void clrb( void )
{
	B=0;
	CLR_NZVC; SEZ;
}

/* $60 NEG indexed ?**** */
M6800_INLINE void neg_ix( void )
{
	UINT16 r,t;
	IDXBYTE(t); r=-t;
	CLR_NZVC; SET_FLAGS8(0,t,r);
	WM(EAD,r);
}

/* $61 AIM --**0- */ /* HD63701YO only */
M6800_INLINE void aim_ix( void )
{
	UINT8 t, r;
	IMMBYTE(t);
	IDXBYTE(r);
	r &= t;
	CLR_NZV; SET_NZ8(r);
	WM(EAD,r);
}

/* $62 OIM --**0- */ /* HD63701YO only */
M6800_INLINE void oim_ix( void )
{
	UINT8 t, r;
	IMMBYTE(t);
	IDXBYTE(r);
	r |= t;
	CLR_NZV; SET_NZ8(r);
	WM(EAD,r);
}

/* $63 COM indexed -**01 */
M6800_INLINE void com_ix( void )
{
	UINT8 t;
	IDXBYTE(t); t = ~t;
	CLR_NZV; SET_NZ8(t); SEC;
	WM(EAD,t);
}

/* $64 LSR indexed -0*-* */
M6800_INLINE void lsr_ix( void )
{
	UINT8 t;
	IDXBYTE(t); CLR_NZC; CC|=(t&0x01);
	t>>=1; SET_Z8(t);
	WM(EAD,t);
}

/* $65 EIM --**0- */ /* HD63701YO only */
M6800_INLINE void eim_ix( void )
{
	UINT8 t, r;
	IMMBYTE(t);
	IDXBYTE(r);
	r ^= t;
	CLR_NZV; SET_NZ8(r);
	WM(EAD,r);
}

/* $66 ROR indexed -**-* */
M6800_INLINE void ror_ix( void )
{
	UINT8 t,r;
	IDXBYTE(t); r=(CC&0x01)<<7;
	CLR_NZC; CC|=(t&0x01);
	r |= t>>1; SET_NZ8(r);
	WM(EAD,r);
}

/* $67 ASR indexed ?**-* */
M6800_INLINE void asr_ix( void )
{
	UINT8 t;
	IDXBYTE(t); CLR_NZC; CC|=(t&0x01);
	t>>=1; t|=((t&0x40)<<1);
	SET_NZ8(t);
	WM(EAD,t);
}

/* $68 ASL indexed ?**** */
M6800_INLINE void asl_ix( void )
{
	UINT16 t,r;
	IDXBYTE(t); r=t<<1;
	CLR_NZVC; SET_FLAGS8(t,t,r);
	WM(EAD,r);
}

/* $69 ROL indexed -**** */
M6800_INLINE void rol_ix( void )
{
	UINT16 t,r;
	IDXBYTE(t); r = CC&0x01; r |= t<<1;
	CLR_NZVC; SET_FLAGS8(t,t,r);
	WM(EAD,r);
}

/* $6a DEC indexed -***- */
M6800_INLINE void dec_ix( void )
{
	UINT8 t;
	IDXBYTE(t); --t;
	CLR_NZV; SET_FLAGS8D(t);
	WM(EAD,t);
}

/* $6b TIM --**0- */ /* HD63701YO only */
M6800_INLINE void tim_ix( void )
{
	UINT8 t, r;
	IMMBYTE(t);
	IDXBYTE(r);
	r &= t;
	CLR_NZV; SET_NZ8(r);
}

/* $6c INC indexed -***- */
M6800_INLINE void inc_ix( void )
{
	UINT8 t;
	IDXBYTE(t); ++t;
	CLR_NZV; SET_FLAGS8I(t);
	WM(EAD,t);
}

/* $6d TST indexed -**0- */
M6800_INLINE void tst_ix( void )
{
	UINT8 t;
	IDXBYTE(t); CLR_NZVC; SET_NZ8(t);
}

/* $6e JMP indexed ----- */
M6800_INLINE void jmp_ix( void )
{
	INDEXED; PC=EA; CHANGE_PC();
}

/* $6f CLR indexed -0100 */
M6800_INLINE void clr_ix( void )
{
	INDEXED; WM(EAD,0);
	CLR_NZVC; SEZ;
}

/* $70 NEG extended ?**** */
M6800_INLINE void neg_ex( void )
{
	UINT16 r,t;
	EXTBYTE(t); r=-t;
	CLR_NZVC; SET_FLAGS8(0,t,r);
	WM(EAD,r);
}

/* $71 AIM --**0- */ /* HD63701YO only */
M6800_INLINE void aim_di( void )
{
	UINT8 t, r;
	IMMBYTE(t);
	DIRBYTE(r);
	r &= t;
	CLR_NZV; SET_NZ8(r);
	WM(EAD,r);
}

/* $72 OIM --**0- */ /* HD63701YO only */
M6800_INLINE void oim_di( void )
{
	UINT8 t, r;
	IMMBYTE(t);
	DIRBYTE(r);
	r |= t;
	CLR_NZV; SET_NZ8(r);
	WM(EAD,r);
}

/* $73 COM extended -**01 */
M6800_INLINE void com_ex( void )
{
	UINT8 t;
	EXTBYTE(t); t = ~t;
	CLR_NZV; SET_NZ8(t); SEC;
	WM(EAD,t);
}

/* $74 LSR extended -0*-* */
M6800_INLINE void lsr_ex( void )
{
	UINT8 t;
	EXTBYTE(t);
	CLR_NZC;
	CC|=(t&0x01);
	t>>=1;
	SET_Z8(t);
	WM(EAD,t);
}

/* $75 EIM --**0- */ /* HD63701YO only */
M6800_INLINE void eim_di( void )
{
	UINT8 t, r;
	IMMBYTE(t);
	DIRBYTE(r);
	r ^= t;
	CLR_NZV; SET_NZ8(r);
	WM(EAD,r);
}

/* $76 ROR extended -**-* */
M6800_INLINE void ror_ex( void )
{
	UINT8 t,r;
	EXTBYTE(t); r=(CC&0x01)<<7;
	CLR_NZC; CC|=(t&0x01);
	r |= t>>1; SET_NZ8(r);
	WM(EAD,r);
}

/* $77 ASR extended ?**-* */
M6800_INLINE void asr_ex( void )
{
	UINT8 t;
	EXTBYTE(t); CLR_NZC; CC|=(t&0x01);
	t>>=1; t|=((t&0x40)<<1);
	SET_NZ8(t);
	WM(EAD,t);
}

/* $78 ASL extended ?**** */
M6800_INLINE void asl_ex( void )
{
	UINT16 t,r;
	EXTBYTE(t); r=t<<1;
	CLR_NZVC; SET_FLAGS8(t,t,r);
	WM(EAD,r);
}

/* $79 ROL extended -**** */
M6800_INLINE void rol_ex( void )
{
	UINT16 t,r;
	EXTBYTE(t); r = CC&0x01; r |= t<<1;
	CLR_NZVC; SET_FLAGS8(t,t,r);
	WM(EAD,r);
}

/* $7a DEC extended -***- */
M6800_INLINE void dec_ex( void )
{
	UINT8 t;
	EXTBYTE(t); --t;
	CLR_NZV; SET_FLAGS8D(t);
	WM(EAD,t);
}

/* $7b TIM --**0- */ /* HD63701YO only */
M6800_INLINE void tim_di( void )
{
	UINT8 t, r;
	IMMBYTE(t);
	DIRBYTE(r);
	r &= t;
	CLR_NZV; SET_NZ8(r);
}

/* $7c INC extended -***- */
M6800_INLINE void inc_ex( void )
{
	UINT8 t;
	EXTBYTE(t); ++t;
	CLR_NZV; SET_FLAGS8I(t);
	WM(EAD,t);
}

/* $7d TST extended -**0- */
M6800_INLINE void tst_ex( void )
{
	UINT8 t;
	EXTBYTE(t); CLR_NZVC; SET_NZ8(t);
}

/* $7e JMP extended ----- */
M6800_INLINE void jmp_ex( void )
{
	EXTENDED; PC=EA; CHANGE_PC(); /* TS 971002 */
}

/* $7f CLR extended -0100 */
M6800_INLINE void clr_ex( void )
{
	EXTENDED; WM(EAD,0);
	CLR_NZVC; SEZ;
}

/* $80 SUBA immediate ?**** */
M6800_INLINE void suba_im( void )
{
	UINT16	  t,r;
	IMMBYTE(t); r = A-t;
	CLR_NZVC; SET_FLAGS8(A,t,r);
	A = r;
}

/* $81 CMPA immediate ?**** */
M6800_INLINE void cmpa_im( void )
{
	UINT16	  t,r;
	IMMBYTE(t); r = A-t;
	CLR_NZVC; SET_FLAGS8(A,t,r);
}

/* $82 SBCA immediate ?**** */
M6800_INLINE void sbca_im( void )
{
	UINT16	  t,r;
	IMMBYTE(t); r = A-t-(CC&0x01);
	CLR_NZVC; SET_FLAGS8(A,t,r);
	A = r;
}

/* $83 SUBD immediate -**** */
M6800_INLINE void subd_im( void )
{
	UINT32 r,d;
	PAIR b;
	IMMWORD(b);
	d = D;
	r = d - b.d;
	CLR_NZVC;
	SET_FLAGS16(d,b.d,r);
	D = r;
}

/* $84 ANDA immediate -**0- */
M6800_INLINE void anda_im( void )
{
	UINT8 t;
	IMMBYTE(t); A &= t;
	CLR_NZV; SET_NZ8(A);
}

/* $85 BITA immediate -**0- */
M6800_INLINE void bita_im( void )
{
	UINT8 t,r;
	IMMBYTE(t); r = A&t;
	CLR_NZV; SET_NZ8(r);
}

/* $86 LDA immediate -**0- */
M6800_INLINE void lda_im( void )
{
	IMMBYTE(A);
	CLR_NZV; SET_NZ8(A);
}

/* is this a legal instruction? */
/* $87 STA immediate -**0- */
M6800_INLINE void sta_im( void )
{
	CLR_NZV; SET_NZ8(A);
	IMM8; WM(EAD,A);
}

/* $88 EORA immediate -**0- */
M6800_INLINE void eora_im( void )
{
	UINT8 t;
	IMMBYTE(t); A ^= t;
	CLR_NZV; SET_NZ8(A);
}

/* $89 ADCA immediate ***** */
M6800_INLINE void adca_im( void )
{
	UINT16 t,r;
	IMMBYTE(t); r = A+t+(CC&0x01);
	CLR_HNZVC; SET_FLAGS8(A,t,r); SET_H(A,t,r);
	A = r;
}

/* $8a ORA immediate -**0- */
M6800_INLINE void ora_im( void )
{
	UINT8 t;
	IMMBYTE(t); A |= t;
	CLR_NZV; SET_NZ8(A);
}

/* $8b ADDA immediate ***** */
M6800_INLINE void adda_im( void )
{
	UINT16 t,r;
	IMMBYTE(t); r = A+t;
	CLR_HNZVC; SET_FLAGS8(A,t,r); SET_H(A,t,r);
	A = r;
}

/* $8c CMPX immediate -***- */
M6800_INLINE void cmpx_im( void )
{
	UINT32 r,d;
	PAIR b;
	IMMWORD(b);
	d = X;
	r = d - b.d;
	CLR_NZV;
	SET_NZ16(r); SET_V16(d,b.d,r);
}

/* $8c CPX immediate -**** (6803) */
M6800_INLINE void cpx_im( void )
{
	UINT32 r,d;
	PAIR b;
	IMMWORD(b);
	d = X;
	r = d - b.d;
	CLR_NZVC; SET_FLAGS16(d,b.d,r);
}


/* $8d BSR ----- */
M6800_INLINE void bsr( void )
{
	UINT8 t;
	IMMBYTE(t);
	PUSHWORD(pPC);
	PC += SIGNED(t);
	CHANGE_PC();	 /* TS 971002 */
}

/* $8e LDS immediate -**0- */
M6800_INLINE void lds_im( void )
{
	IMMWORD(m6808.s);
	CLR_NZV;
	SET_NZ16(S);
}

/* $8f STS immediate -**0- */
M6800_INLINE void sts_im( void )
{
	CLR_NZV;
	SET_NZ16(S);
	IMM16;
	WM16(EAD,&m6808.s);
}

/* $90 SUBA direct ?**** */
M6800_INLINE void suba_di( void )
{
	UINT16	  t,r;
	DIRBYTE(t); r = A-t;
	CLR_NZVC; SET_FLAGS8(A,t,r);
	A = r;
}

/* $91 CMPA direct ?**** */
M6800_INLINE void cmpa_di( void )
{
	UINT16	  t,r;
	DIRBYTE(t); r = A-t;
	CLR_NZVC; SET_FLAGS8(A,t,r);
}

/* $92 SBCA direct ?**** */
M6800_INLINE void sbca_di( void )
{
	UINT16	  t,r;
	DIRBYTE(t); r = A-t-(CC&0x01);
	CLR_NZVC; SET_FLAGS8(A,t,r);
	A = r;
}

/* $93 SUBD direct -**** */
M6800_INLINE void subd_di( void )
{
	UINT32 r,d;
	PAIR b;
	DIRWORD(b);
	d = D;
	r = d - b.d;
	CLR_NZVC;
	SET_FLAGS16(d,b.d,r);
	D=r;
}

/* $94 ANDA direct -**0- */
M6800_INLINE void anda_di( void )
{
	UINT8 t;
	DIRBYTE(t); A &= t;
	CLR_NZV; SET_NZ8(A);
}

/* $95 BITA direct -**0- */
M6800_INLINE void bita_di( void )
{
	UINT8 t,r;
	DIRBYTE(t); r = A&t;
	CLR_NZV; SET_NZ8(r);
}

/* $96 LDA direct -**0- */
M6800_INLINE void lda_di( void )
{
	DIRBYTE(A);
	CLR_NZV; SET_NZ8(A);
}

/* $97 STA direct -**0- */
M6800_INLINE void sta_di( void )
{
	CLR_NZV; SET_NZ8(A);
	DIRECT; WM(EAD,A);
}

/* $98 EORA direct -**0- */
M6800_INLINE void eora_di( void )
{
	UINT8 t;
	DIRBYTE(t); A ^= t;
	CLR_NZV; SET_NZ8(A);
}

/* $99 ADCA direct ***** */
M6800_INLINE void adca_di( void )
{
	UINT16 t,r;
	DIRBYTE(t); r = A+t+(CC&0x01);
	CLR_HNZVC; SET_FLAGS8(A,t,r); SET_H(A,t,r);
	A = r;
}

/* $9a ORA direct -**0- */
M6800_INLINE void ora_di( void )
{
	UINT8 t;
	DIRBYTE(t); A |= t;
	CLR_NZV; SET_NZ8(A);
}

/* $9b ADDA direct ***** */
M6800_INLINE void adda_di( void )
{
	UINT16 t,r;
	DIRBYTE(t); r = A+t;
	CLR_HNZVC; SET_FLAGS8(A,t,r); SET_H(A,t,r);
	A = r;
}

/* $9c CMPX direct -***- */
M6800_INLINE void cmpx_di( void )
{
	UINT32 r,d;
	PAIR b;
	DIRWORD(b);
	d = X;
	r = d - b.d;
	CLR_NZV;
	SET_NZ16(r); SET_V16(d,b.d,r);
}

/* $9c CPX direct -**** (6803) */
M6800_INLINE void cpx_di( void )
{
	UINT32 r,d;
	PAIR b;
	DIRWORD(b);
	d = X;
	r = d - b.d;
	CLR_NZVC; SET_FLAGS16(d,b.d,r);
}

/* $9d JSR direct ----- */
M6800_INLINE void jsr_di( void )
{
	DIRECT;
	PUSHWORD(pPC);
    PC = EA;
	CHANGE_PC();
}

/* $9e LDS direct -**0- */
M6800_INLINE void lds_di( void )
{
	DIRWORD(m6808.s);
	CLR_NZV;
	SET_NZ16(S);
}

/* $9f STS direct -**0- */
M6800_INLINE void sts_di( void )
{
	CLR_NZV;
	SET_NZ16(S);
	DIRECT;
	WM16(EAD,&m6808.s);
}

/* $a0 SUBA indexed ?**** */
M6800_INLINE void suba_ix( void )
{
	UINT16	  t,r;
	IDXBYTE(t); r = A-t;
	CLR_NZVC; SET_FLAGS8(A,t,r);
	A = r;
}

/* $a1 CMPA indexed ?**** */
M6800_INLINE void cmpa_ix( void )
{
	UINT16	  t,r;
	IDXBYTE(t); r = A-t;
	CLR_NZVC; SET_FLAGS8(A,t,r);
}

/* $a2 SBCA indexed ?**** */
M6800_INLINE void sbca_ix( void )
{
	UINT16	  t,r;
	IDXBYTE(t); r = A-t-(CC&0x01);
	CLR_NZVC; SET_FLAGS8(A,t,r);
	A = r;
}

/* $a3 SUBD indexed -**** */
M6800_INLINE void subd_ix( void )
{
	UINT32 r,d;
	PAIR b;
	IDXWORD(b);
	d = D;
	r = d - b.d;
	CLR_NZVC;
	SET_FLAGS16(d,b.d,r);
	D = r;
}

/* $a4 ANDA indexed -**0- */
M6800_INLINE void anda_ix( void )
{
	UINT8 t;
	IDXBYTE(t); A &= t;
	CLR_NZV; SET_NZ8(A);
}

/* $a5 BITA indexed -**0- */
M6800_INLINE void bita_ix( void )
{
	UINT8 t,r;
	IDXBYTE(t); r = A&t;
	CLR_NZV; SET_NZ8(r);
}

/* $a6 LDA indexed -**0- */
M6800_INLINE void lda_ix( void )
{
	IDXBYTE(A);
	CLR_NZV; SET_NZ8(A);
}

/* $a7 STA indexed -**0- */
M6800_INLINE void sta_ix( void )
{
	CLR_NZV; SET_NZ8(A);
	INDEXED; WM(EAD,A);
}

/* $a8 EORA indexed -**0- */
M6800_INLINE void eora_ix( void )
{
	UINT8 t;
	IDXBYTE(t); A ^= t;
	CLR_NZV; SET_NZ8(A);
}

/* $a9 ADCA indexed ***** */
M6800_INLINE void adca_ix( void )
{
	UINT16 t,r;
	IDXBYTE(t); r = A+t+(CC&0x01);
	CLR_HNZVC; SET_FLAGS8(A,t,r); SET_H(A,t,r);
	A = r;
}

/* $aa ORA indexed -**0- */
M6800_INLINE void ora_ix( void )
{
	UINT8 t;
	IDXBYTE(t); A |= t;
	CLR_NZV; SET_NZ8(A);
}

/* $ab ADDA indexed ***** */
M6800_INLINE void adda_ix( void )
{
	UINT16 t,r;
	IDXBYTE(t); r = A+t;
	CLR_HNZVC; SET_FLAGS8(A,t,r); SET_H(A,t,r);
	A = r;
}

/* $ac CMPX indexed -***- */
M6800_INLINE void cmpx_ix( void )
{
	UINT32 r,d;
	PAIR b;
	IDXWORD(b);
	d = X;
	r = d - b.d;
	CLR_NZV;
	SET_NZ16(r); SET_V16(d,b.d,r);
}

/* $ac CPX indexed -**** (6803)*/
M6800_INLINE void cpx_ix( void )
{
	UINT32 r,d;
	PAIR b;
	IDXWORD(b);
	d = X;
	r = d - b.d;
	CLR_NZVC; SET_FLAGS16(d,b.d,r);
}

/* $ad JSR indexed ----- */
M6800_INLINE void jsr_ix( void )
{
	INDEXED;
	PUSHWORD(pPC);
    PC = EA;
	CHANGE_PC();
}

/* $ae LDS indexed -**0- */
M6800_INLINE void lds_ix( void )
{
	IDXWORD(m6808.s);
	CLR_NZV;
	SET_NZ16(S);
}

/* $af STS indexed -**0- */
M6800_INLINE void sts_ix( void )
{
	CLR_NZV;
	SET_NZ16(S);
	INDEXED;
	WM16(EAD,&m6808.s);
}

/* $b0 SUBA extended ?**** */
M6800_INLINE void suba_ex( void )
{
	UINT16	  t,r;
	EXTBYTE(t); r = A-t;
	CLR_NZVC; SET_FLAGS8(A,t,r);
	A = r;
}

/* $b1 CMPA extended ?**** */
M6800_INLINE void cmpa_ex( void )
{
	UINT16	  t,r;
	EXTBYTE(t); r = A-t;
	CLR_NZVC; SET_FLAGS8(A,t,r);
}

/* $b2 SBCA extended ?**** */
M6800_INLINE void sbca_ex( void )
{
	UINT16	  t,r;
	EXTBYTE(t); r = A-t-(CC&0x01);
	CLR_NZVC; SET_FLAGS8(A,t,r);
	A = r;
}

/* $b3 SUBD extended -**** */
M6800_INLINE void subd_ex( void )
{
	UINT32 r,d;
	PAIR b;
	EXTWORD(b);
	d = D;
	r = d - b.d;
	CLR_NZVC;
	SET_FLAGS16(d,b.d,r);
	D=r;
}

/* $b4 ANDA extended -**0- */
M6800_INLINE void anda_ex( void )
{
	UINT8 t;
	EXTBYTE(t); A &= t;
	CLR_NZV; SET_NZ8(A);
}

/* $b5 BITA extended -**0- */
M6800_INLINE void bita_ex( void )
{
	UINT8 t,r;
	EXTBYTE(t); r = A&t;
	CLR_NZV; SET_NZ8(r);
}

/* $b6 LDA extended -**0- */
M6800_INLINE void lda_ex( void )
{
	EXTBYTE(A);
	CLR_NZV; SET_NZ8(A);
}

/* $b7 STA extended -**0- */
M6800_INLINE void sta_ex( void )
{
	CLR_NZV; SET_NZ8(A);
	EXTENDED; WM(EAD,A);
}

/* $b8 EORA extended -**0- */
M6800_INLINE void eora_ex( void )
{
	UINT8 t;
	EXTBYTE(t); A ^= t;
	CLR_NZV; SET_NZ8(A);
}

/* $b9 ADCA extended ***** */
M6800_INLINE void adca_ex( void )
{
	UINT16 t,r;
	EXTBYTE(t); r = A+t+(CC&0x01);
	CLR_HNZVC; SET_FLAGS8(A,t,r); SET_H(A,t,r);
	A = r;
}

/* $ba ORA extended -**0- */
M6800_INLINE void ora_ex( void )
{
	UINT8 t;
	EXTBYTE(t); A |= t;
	CLR_NZV; SET_NZ8(A);
}

/* $bb ADDA extended ***** */
M6800_INLINE void adda_ex( void )
{
	UINT16 t,r;
	EXTBYTE(t); r = A+t;
	CLR_HNZVC; SET_FLAGS8(A,t,r); SET_H(A,t,r);
	A = r;
}

/* $bc CMPX extended -***- */
M6800_INLINE void cmpx_ex( void )
{
	UINT32 r,d;
	PAIR b;
	EXTWORD(b);
	d = X;
	r = d - b.d;
	CLR_NZV;
	SET_NZ16(r); SET_V16(d,b.d,r);
}

/* $bc CPX extended -**** (6803) */
M6800_INLINE void cpx_ex( void )
{
	UINT32 r,d;
	PAIR b;
	EXTWORD(b);
	d = X;
	r = d - b.d;
	CLR_NZVC; SET_FLAGS16(d,b.d,r);
}

/* $bd JSR extended ----- */
M6800_INLINE void jsr_ex( void )
{
	EXTENDED;
	PUSHWORD(pPC);
    PC = EA;
	CHANGE_PC();
}

/* $be LDS extended -**0- */
M6800_INLINE void lds_ex( void )
{
	EXTWORD(m6808.s);
	CLR_NZV;
	SET_NZ16(S);
}

/* $bf STS extended -**0- */
M6800_INLINE void sts_ex( void )
{
	CLR_NZV;
	SET_NZ16(S);
	EXTENDED;
	WM16(EAD,&m6808.s);
}

/* $c0 SUBB immediate ?**** */
M6800_INLINE void subb_im( void )
{
	UINT16	  t,r;
	IMMBYTE(t); r = B-t;
	CLR_NZVC; SET_FLAGS8(B,t,r);
	B = r;
}

/* $c1 CMPB immediate ?**** */
M6800_INLINE void cmpb_im( void )
{
	UINT16	  t,r;
	IMMBYTE(t); r = B-t;
	CLR_NZVC; SET_FLAGS8(B,t,r);
}

/* $c2 SBCB immediate ?**** */
M6800_INLINE void sbcb_im( void )
{
	UINT16	  t,r;
	IMMBYTE(t); r = B-t-(CC&0x01);
	CLR_NZVC; SET_FLAGS8(B,t,r);
	B = r;
}

/* $c3 ADDD immediate -**** */
M6800_INLINE void addd_im( void )
{
	UINT32 r,d;
	PAIR b;
	IMMWORD(b);
	d = D;
	r = d + b.d;
	CLR_NZVC;
	SET_FLAGS16(d,b.d,r);
	D = r;
}

/* $c4 ANDB immediate -**0- */
M6800_INLINE void andb_im( void )
{
	UINT8 t;
	IMMBYTE(t); B &= t;
	CLR_NZV; SET_NZ8(B);
}

/* $c5 BITB immediate -**0- */
M6800_INLINE void bitb_im( void )
{
	UINT8 t,r;
	IMMBYTE(t); r = B&t;
	CLR_NZV; SET_NZ8(r);
}

/* $c6 LDB immediate -**0- */
M6800_INLINE void ldb_im( void )
{
	IMMBYTE(B);
	CLR_NZV; SET_NZ8(B);
}

/* is this a legal instruction? */
/* $c7 STB immediate -**0- */
M6800_INLINE void stb_im( void )
{
	CLR_NZV; SET_NZ8(B);
	IMM8; WM(EAD,B);
}

/* $c8 EORB immediate -**0- */
M6800_INLINE void eorb_im( void )
{
	UINT8 t;
	IMMBYTE(t); B ^= t;
	CLR_NZV; SET_NZ8(B);
}

/* $c9 ADCB immediate ***** */
M6800_INLINE void adcb_im( void )
{
	UINT16 t,r;
	IMMBYTE(t); r = B+t+(CC&0x01);
	CLR_HNZVC; SET_FLAGS8(B,t,r); SET_H(B,t,r);
	B = r;
}

/* $ca ORB immediate -**0- */
M6800_INLINE void orb_im( void )
{
	UINT8 t;
	IMMBYTE(t); B |= t;
	CLR_NZV; SET_NZ8(B);
}

/* $cb ADDB immediate ***** */
M6800_INLINE void addb_im( void )
{
	UINT16 t,r;
	IMMBYTE(t); r = B+t;
	CLR_HNZVC; SET_FLAGS8(B,t,r); SET_H(B,t,r);
	B = r;
}

/* $CC LDD immediate -**0- */
M6800_INLINE void ldd_im( void )
{
	IMMWORD(m6808.d);
	CLR_NZV;
	SET_NZ16(D);
}

/* is this a legal instruction? */
/* $cd STD immediate -**0- */
M6800_INLINE void std_im( void )
{
	IMM16;
	CLR_NZV;
	SET_NZ16(D);
	WM16(EAD,&m6808.d);
}

/* $ce LDX immediate -**0- */
M6800_INLINE void ldx_im( void )
{
	IMMWORD(m6808.x);
	CLR_NZV;
	SET_NZ16(X);
}

/* $cf STX immediate -**0- */
M6800_INLINE void stx_im( void )
{
	CLR_NZV;
	SET_NZ16(X);
	IMM16;
	WM16(EAD,&m6808.x);
}

/* $d0 SUBB direct ?**** */
M6800_INLINE void subb_di( void )
{
	UINT16	  t,r;
	DIRBYTE(t); r = B-t;
	CLR_NZVC; SET_FLAGS8(B,t,r);
	B = r;
}

/* $d1 CMPB direct ?**** */
M6800_INLINE void cmpb_di( void )
{
	UINT16	  t,r;
	DIRBYTE(t); r = B-t;
	CLR_NZVC; SET_FLAGS8(B,t,r);
}

/* $d2 SBCB direct ?**** */
M6800_INLINE void sbcb_di( void )
{
	UINT16	  t,r;
	DIRBYTE(t); r = B-t-(CC&0x01);
	CLR_NZVC; SET_FLAGS8(B,t,r);
	B = r;
}

/* $d3 ADDD direct -**** */
M6800_INLINE void addd_di( void )
{
	UINT32 r,d;
	PAIR b;
	DIRWORD(b);
	d = D;
	r = d + b.d;
	CLR_NZVC;
	SET_FLAGS16(d,b.d,r);
	D = r;
}

/* $d4 ANDB direct -**0- */
M6800_INLINE void andb_di( void )
{
	UINT8 t;
	DIRBYTE(t); B &= t;
	CLR_NZV; SET_NZ8(B);
}

/* $d5 BITB direct -**0- */
M6800_INLINE void bitb_di( void )
{
	UINT8 t,r;
	DIRBYTE(t); r = B&t;
	CLR_NZV; SET_NZ8(r);
}

/* $d6 LDB direct -**0- */
M6800_INLINE void ldb_di( void )
{
	DIRBYTE(B);
	CLR_NZV; SET_NZ8(B);
}

/* $d7 STB direct -**0- */
M6800_INLINE void stb_di( void )
{
	CLR_NZV; SET_NZ8(B);
	DIRECT; WM(EAD,B);
}

/* $d8 EORB direct -**0- */
M6800_INLINE void eorb_di( void )
{
	UINT8 t;
	DIRBYTE(t); B ^= t;
	CLR_NZV; SET_NZ8(B);
}

/* $d9 ADCB direct ***** */
M6800_INLINE void adcb_di( void )
{
	UINT16 t,r;
	DIRBYTE(t); r = B+t+(CC&0x01);
	CLR_HNZVC; SET_FLAGS8(B,t,r); SET_H(B,t,r);
	B = r;
}

/* $da ORB direct -**0- */
M6800_INLINE void orb_di( void )
{
	UINT8 t;
	DIRBYTE(t); B |= t;
	CLR_NZV; SET_NZ8(B);
}

/* $db ADDB direct ***** */
M6800_INLINE void addb_di( void )
{
	UINT16 t,r;
	DIRBYTE(t); r = B+t;
	CLR_HNZVC; SET_FLAGS8(B,t,r); SET_H(B,t,r);
	B = r;
}

/* $dc LDD direct -**0- */
M6800_INLINE void ldd_di( void )
{
	DIRWORD(m6808.d);
	CLR_NZV;
	SET_NZ16(D);
}

/* $dd STD direct -**0- */
M6800_INLINE void std_di( void )
{
	DIRECT;
	CLR_NZV;
	SET_NZ16(D);
	WM16(EAD,&m6808.d);
}

/* $de LDX direct -**0- */
M6800_INLINE void ldx_di( void )
{
	DIRWORD(m6808.x);
	CLR_NZV;
	SET_NZ16(X);
}

/* $dF STX direct -**0- */
M6800_INLINE void stx_di( void )
{
	CLR_NZV;
	SET_NZ16(X);
	DIRECT;
	WM16(EAD,&m6808.x);
}

/* $e0 SUBB indexed ?**** */
M6800_INLINE void subb_ix( void )
{
	UINT16	  t,r;
	IDXBYTE(t); r = B-t;
	CLR_NZVC; SET_FLAGS8(B,t,r);
	B = r;
}

/* $e1 CMPB indexed ?**** */
M6800_INLINE void cmpb_ix( void )
{
	UINT16	  t,r;
	IDXBYTE(t); r = B-t;
	CLR_NZVC; SET_FLAGS8(B,t,r);
}

/* $e2 SBCB indexed ?**** */
M6800_INLINE void sbcb_ix( void )
{
	UINT16	  t,r;
	IDXBYTE(t); r = B-t-(CC&0x01);
	CLR_NZVC; SET_FLAGS8(B,t,r);
	B = r;
}

/* $e3 ADDD indexed -**** */
M6800_INLINE void addd_ix( void )
{
	UINT32 r,d;
	PAIR b;
	IDXWORD(b);
	d = D;
	r = d + b.d;
	CLR_NZVC;
	SET_FLAGS16(d,b.d,r);
	D = r;
}

/* $e4 ANDB indexed -**0- */
M6800_INLINE void andb_ix( void )
{
	UINT8 t;
	IDXBYTE(t); B &= t;
	CLR_NZV; SET_NZ8(B);
}

/* $e5 BITB indexed -**0- */
M6800_INLINE void bitb_ix( void )
{
	UINT8 t,r;
	IDXBYTE(t); r = B&t;
	CLR_NZV; SET_NZ8(r);
}

/* $e6 LDB indexed -**0- */
M6800_INLINE void ldb_ix( void )
{
	IDXBYTE(B);
	CLR_NZV; SET_NZ8(B);
}

/* $e7 STB indexed -**0- */
M6800_INLINE void stb_ix( void )
{
	CLR_NZV; SET_NZ8(B);
	INDEXED; WM(EAD,B);
}

/* $e8 EORB indexed -**0- */
M6800_INLINE void eorb_ix( void )
{
	UINT8 t;
	IDXBYTE(t); B ^= t;
	CLR_NZV; SET_NZ8(B);
}

/* $e9 ADCB indexed ***** */
M6800_INLINE void adcb_ix( void )
{
	UINT16 t,r;
	IDXBYTE(t); r = B+t+(CC&0x01);
	CLR_HNZVC; SET_FLAGS8(B,t,r); SET_H(B,t,r);
	B = r;
}

/* $ea ORB indexed -**0- */
M6800_INLINE void orb_ix( void )
{
	UINT8 t;
	IDXBYTE(t); B |= t;
	CLR_NZV; SET_NZ8(B);
}

/* $eb ADDB indexed ***** */
M6800_INLINE void addb_ix( void )
{
	UINT16 t,r;
	IDXBYTE(t); r = B+t;
	CLR_HNZVC; SET_FLAGS8(B,t,r); SET_H(B,t,r);
	B = r;
}

/* $ec LDD indexed -**0- */
M6800_INLINE void ldd_ix( void )
{
	IDXWORD(m6808.d);
	CLR_NZV;
	SET_NZ16(D);
}

#if 0
/* $ec ADCX immediate -****    NSC8105 only.  Flags are a guess - copied from addb_im() */
M6800_INLINE void adcx_im( void )
{
	UINT16 t,r;
	IMMBYTE(t); r = X+t+(CC&0x01);
	CLR_HNZVC; SET_FLAGS8(X,t,r); SET_H(X,t,r);
	X = r;
}
#endif

/* $ed STD indexed -**0- */
M6800_INLINE void std_ix( void )
{
	INDEXED;
	CLR_NZV;
	SET_NZ16(D);
	WM16(EAD,&m6808.d);
}

/* $ee LDX indexed -**0- */
M6800_INLINE void ldx_ix( void )
{
	IDXWORD(m6808.x);
	CLR_NZV;
	SET_NZ16(X);
}

/* $ef STX indexed -**0- */
M6800_INLINE void stx_ix( void )
{
	CLR_NZV;
	SET_NZ16(X);
	INDEXED;
	WM16(EAD,&m6808.x);
}

/* $f0 SUBB extended ?**** */
M6800_INLINE void subb_ex( void )
{
	UINT16	  t,r;
	EXTBYTE(t); r = B-t;
	CLR_NZVC; SET_FLAGS8(B,t,r);
	B = r;
}

/* $f1 CMPB extended ?**** */
M6800_INLINE void cmpb_ex( void )
{
	UINT16	  t,r;
	EXTBYTE(t); r = B-t;
	CLR_NZVC; SET_FLAGS8(B,t,r);
}

/* $f2 SBCB extended ?**** */
M6800_INLINE void sbcb_ex( void )
{
	UINT16	  t,r;
	EXTBYTE(t); r = B-t-(CC&0x01);
	CLR_NZVC; SET_FLAGS8(B,t,r);
	B = r;
}

/* $f3 ADDD extended -**** */
M6800_INLINE void addd_ex( void )
{
	UINT32 r,d;
	PAIR b;
	EXTWORD(b);
	d = D;
	r = d + b.d;
	CLR_NZVC;
	SET_FLAGS16(d,b.d,r);
	D = r;
}

/* $f4 ANDB extended -**0- */
M6800_INLINE void andb_ex( void )
{
	UINT8 t;
	EXTBYTE(t);
	B &= t;
	CLR_NZV;
	SET_NZ8(B);
}

/* $f5 BITB extended -**0- */
M6800_INLINE void bitb_ex( void )
{
	UINT8 t,r;
	EXTBYTE(t);
	r = B & t;
	CLR_NZV;
	SET_NZ8(r);
}

/* $f6 LDB extended -**0- */
M6800_INLINE void ldb_ex( void )
{
	EXTBYTE(B);
	CLR_NZV;
	SET_NZ8(B);
}

/* $f7 STB extended -**0- */
M6800_INLINE void stb_ex( void )
{
	CLR_NZV; SET_NZ8(B);
	EXTENDED; WM(EAD,B);
}

/* $f8 EORB extended -**0- */
M6800_INLINE void eorb_ex( void )
{
	UINT8 t;
	EXTBYTE(t); B ^= t;
	CLR_NZV; SET_NZ8(B);
}

/* $f9 ADCB extended ***** */
M6800_INLINE void adcb_ex( void )
{
	UINT16 t,r;
	EXTBYTE(t); r = B+t+(CC&0x01);
	CLR_HNZVC; SET_FLAGS8(B,t,r); SET_H(B,t,r);
	B = r;
}

/* $fa ORB extended -**0- */
M6800_INLINE void orb_ex( void )
{
	UINT8 t;
	EXTBYTE(t); B |= t;
	CLR_NZV; SET_NZ8(B);
}

/* $fb ADDB extended ***** */
M6800_INLINE void addb_ex( void )
{
	UINT16 t,r;
	EXTBYTE(t); r = B+t;
	CLR_HNZVC; SET_FLAGS8(B,t,r); SET_H(B,t,r);
	B = r;
}

/* $fc LDD extended -**0- */
M6800_INLINE void ldd_ex( void )
{
	EXTWORD(m6808.d);
	CLR_NZV;
	SET_NZ16(D);
}

/* $fc ADDX extended -****    NSC8105 only.  Flags are a guess */
M6800_INLINE void addx_ex( void )
{
	UINT32 r,d;
	PAIR b;
	EXTWORD(b);
	d = X;
	r = d + b.d;
	CLR_NZVC;
	SET_FLAGS16(d,b.d,r);
	X = r;
}

/* $fd STD extended -**0- */
M6800_INLINE void std_ex( void )
{
	EXTENDED;
	CLR_NZV;
	SET_NZ16(D);
	WM16(EAD,&m6808.d);
}

/* $fe LDX extended -**0- */
M6800_INLINE void ldx_ex( void )
{
	EXTWORD(m6808.x);
	CLR_NZV;
	SET_NZ16(X);
}

/* $ff STX extended -**0- */
M6800_INLINE void stx_ex( void )
{
	CLR_NZV;
	SET_NZ16(X);
	EXTENDED;
	WM16(EAD,&m6808.x);
}


#ifdef __cplusplus
}
#endif

/****************************************************************************
 * Reset registers to their initial values
 ****************************************************************************/

void m6800_init()
{
//  m6800.subtype   = SUBTYPE_M6800;
	m6800.insn = m6800_insn;
	m6800.cycles = cycles_6800;
//	m6800.irq_callback = irqcallback;
//	state_register("m6800", index);
}

void m6800_reset(void)
{
	SEI;				/* IRQ disabled */
	PCD = RM16( 0xfffe );
	CHANGE_PC();

	m6800.wai_state = 0;
	m6800.nmi_state = 0;
	m6800.irq_state[M6800_IRQ_LINE] = 0;
	m6800.irq_state[M6800_TIN_LINE] = 0;
	m6800.ic_eddge = 0;

	m6800.port1_ddr = 0x00;
	m6800.port2_ddr = 0x00;
	/* TODO: on reset port 2 should be read to determine the operating mode (bits 0-2) */
	m6800.tcsr = 0x00;
	m6800.pending_tcsr = 0x00;
	m6800.irq2 = 0;
	CTD = 0x0000;
	OCD = 0xffff;
	TOD = 0xffff;
	m6800.ram_ctrl |= 0x40;
}

int m6800_get_pc()
{
	return PC;
}

/****************************************************************************
 * Shut down CPU emulatio
 ****************************************************************************/
/*
static void m6800_exit(void)
{

}*/

/****************************************************************************
 * Get all registers in given buffer
 ****************************************************************************/
void m6800_get_context(void *dst)
{
	if( dst )
		*(m6800_Regs*)dst = m6800;
}


/****************************************************************************
 * Set all registers to given values
 ****************************************************************************/
void m6800_set_context(void *src)
{
	if( src )
		m6800 = *(m6800_Regs*)src;
	CHANGE_PC();
	CHECK_IRQ_LINES(); /* HJB 990417 */
}


void m6800_set_irq_line(int irqline, int state)
{
	if (irqline == M6800_INPUT_LINE_NMI)
	{
		if (m6800.nmi_state == state) return;
		LOG(("M6800#%d set_nmi_line %d \n", cpu_getactivecpu(), state));
		m6800.nmi_state = state;
		if (state == M6800_CLEAR_LINE) return;

		/* NMI */
		ENTER_INTERRUPT("M6800#%d take NMI\n",0xfffc);
	}
	else
	{
		int eddge;

		if (m6800.irq_state[irqline] == state) return;
		LOG(("M6800#%d set_irq_line %d,%d\n", cpu_getactivecpu(), irqline, state));
		m6800.irq_state[irqline] = state;

		switch(irqline)
		{
		case M6800_IRQ_LINE:
			if (state == M6800_CLEAR_LINE) return;
			break;
		case M6800_TIN_LINE:
			eddge = (state == M6800_CLEAR_LINE ) ? 2 : 0;
			if( ((m6800.tcsr&TCSR_IEDG) ^ (state==M6800_CLEAR_LINE ? TCSR_IEDG : 0))==0 )
				return;
			/* active edge in */
			m6800.tcsr |= TCSR_ICF;
			m6800.pending_tcsr |= TCSR_ICF;
			m6800.input_capture = CT;
			MODIFIED_tcsr;
			if( !(CC & 0x10) )
				CHECK_IRQ2
			break;
		default:
			return;
		}
		CHECK_IRQ_LINES(); /* HJB 990417 */
	}
}

/****************************************************************************
 * Execute cycles CPU cycles. Return number of cycles really executed
 ****************************************************************************/
int m6800_execute(int cycles)
{
	UINT8 ireg;
	m6800_ICount = cycles;

	CLEANUP_conters;
	INCREMENT_COUNTER(m6800.extra_cycles);
	m6800.extra_cycles = 0;

	do
	{
		if( m6800.wai_state & M6800_WAI )
		{
			EAT_CYCLES;
		}
		else
		{
			pPPC = pPC;
//			CALL_MAME_DEBUG;
			ireg=M_RDOP(PCD);
			PC++;

			switch( ireg )
			{
				case 0x00: illegal(); break;
				case 0x01: nop(); break;
				case 0x02: illegal(); break;
				case 0x03: illegal(); break;
				case 0x04: illegal(); break;
				case 0x05: illegal(); break;
				case 0x06: tap(); break;
				case 0x07: tpa(); break;
				case 0x08: inx(); break;
				case 0x09: dex(); break;
				case 0x0A: CLV; break;
				case 0x0B: SEV; break;
				case 0x0C: CLC; break;
				case 0x0D: SEC; break;
				case 0x0E: cli(); break;
				case 0x0F: sei(); break;
				case 0x10: sba(); break;
				case 0x11: cba(); break;
				case 0x12: illegal(); break;
				case 0x13: illegal(); break;
				case 0x14: illegal(); break;
				case 0x15: illegal(); break;
				case 0x16: tab(); break;
				case 0x17: tba(); break;
				case 0x18: illegal(); break;
				case 0x19: daa(); break;
				case 0x1a: illegal(); break;
				case 0x1b: aba(); break;
				case 0x1c: illegal(); break;
				case 0x1d: illegal(); break;
				case 0x1e: illegal(); break;
				case 0x1f: illegal(); break;
				case 0x20: bra(); break;
				case 0x21: brn(); break;
				case 0x22: bhi(); break;
				case 0x23: bls(); break;
				case 0x24: bcc(); break;
				case 0x25: bcs(); break;
				case 0x26: bne(); break;
				case 0x27: beq(); break;
				case 0x28: bvc(); break;
				case 0x29: bvs(); break;
				case 0x2a: bpl(); break;
				case 0x2b: bmi(); break;
				case 0x2c: bge(); break;
				case 0x2d: blt(); break;
				case 0x2e: bgt(); break;
				case 0x2f: ble(); break;
				case 0x30: tsx(); break;
				case 0x31: ins(); break;
				case 0x32: pula(); break;
				case 0x33: pulb(); break;
				case 0x34: des(); break;
				case 0x35: txs(); break;
				case 0x36: psha(); break;
				case 0x37: pshb(); break;
				case 0x38: illegal(); break;
				case 0x39: rts(); break;
				case 0x3a: illegal(); break;
				case 0x3b: rti(); break;
				case 0x3c: illegal(); break;
				case 0x3d: illegal(); break;
				case 0x3e: wai(); break;
				case 0x3f: swi(); break;
				case 0x40: nega(); break;
				case 0x41: illegal(); break;
				case 0x42: illegal(); break;
				case 0x43: coma(); break;
				case 0x44: lsra(); break;
				case 0x45: illegal(); break;
				case 0x46: rora(); break;
				case 0x47: asra(); break;
				case 0x48: asla(); break;
				case 0x49: rola(); break;
				case 0x4a: deca(); break;
				case 0x4b: illegal(); break;
				case 0x4c: inca(); break;
				case 0x4d: tsta(); break;
				case 0x4e: illegal(); break;
				case 0x4f: clra(); break;
				case 0x50: negb(); break;
				case 0x51: illegal(); break;
				case 0x52: illegal(); break;
				case 0x53: comb(); break;
				case 0x54: lsrb(); break;
				case 0x55: illegal(); break;
				case 0x56: rorb(); break;
				case 0x57: asrb(); break;
				case 0x58: aslb(); break;
				case 0x59: rolb(); break;
				case 0x5a: decb(); break;
				case 0x5b: illegal(); break;
				case 0x5c: incb(); break;
				case 0x5d: tstb(); break;
				case 0x5e: illegal(); break;
				case 0x5f: clrb(); break;
				case 0x60: neg_ix(); break;
				case 0x61: illegal(); break;
				case 0x62: illegal(); break;
				case 0x63: com_ix(); break;
				case 0x64: lsr_ix(); break;
				case 0x65: illegal(); break;
				case 0x66: ror_ix(); break;
				case 0x67: asr_ix(); break;
				case 0x68: asl_ix(); break;
				case 0x69: rol_ix(); break;
				case 0x6a: dec_ix(); break;
				case 0x6b: illegal(); break;
				case 0x6c: inc_ix(); break;
				case 0x6d: tst_ix(); break;
				case 0x6e: jmp_ix(); break;
				case 0x6f: clr_ix(); break;
				case 0x70: neg_ex(); break;
				case 0x71: illegal(); break;
				case 0x72: illegal(); break;
				case 0x73: com_ex(); break;
				case 0x74: lsr_ex(); break;
				case 0x75: illegal(); break;
				case 0x76: ror_ex(); break;
				case 0x77: asr_ex(); break;
				case 0x78: asl_ex(); break;
				case 0x79: rol_ex(); break;
				case 0x7a: dec_ex(); break;
				case 0x7b: illegal(); break;
				case 0x7c: inc_ex(); break;
				case 0x7d: tst_ex(); break;
				case 0x7e: jmp_ex(); break;
				case 0x7f: clr_ex(); break;
				case 0x80: suba_im(); break;
				case 0x81: cmpa_im(); break;
				case 0x82: sbca_im(); break;
				case 0x83: illegal(); break;
				case 0x84: anda_im(); break;
				case 0x85: bita_im(); break;
				case 0x86: lda_im(); break;
				case 0x87: sta_im(); break;
				case 0x88: eora_im(); break;
				case 0x89: adca_im(); break;
				case 0x8a: ora_im(); break;
				case 0x8b: adda_im(); break;
				case 0x8c: cmpx_im(); break;
				case 0x8d: bsr(); break;
				case 0x8e: lds_im(); break;
				case 0x8f: sts_im(); /* orthogonality */ break;
				case 0x90: suba_di(); break;
				case 0x91: cmpa_di(); break;
				case 0x92: sbca_di(); break;
				case 0x93: illegal(); break;
				case 0x94: anda_di(); break;
				case 0x95: bita_di(); break;
				case 0x96: lda_di(); break;
				case 0x97: sta_di(); break;
				case 0x98: eora_di(); break;
				case 0x99: adca_di(); break;
				case 0x9a: ora_di(); break;
				case 0x9b: adda_di(); break;
				case 0x9c: cmpx_di(); break;
				case 0x9d: jsr_di(); break;
				case 0x9e: lds_di(); break;
				case 0x9f: sts_di(); break;
				case 0xa0: suba_ix(); break;
				case 0xa1: cmpa_ix(); break;
				case 0xa2: sbca_ix(); break;
				case 0xa3: illegal(); break;
				case 0xa4: anda_ix(); break;
				case 0xa5: bita_ix(); break;
				case 0xa6: lda_ix(); break;
				case 0xa7: sta_ix(); break;
				case 0xa8: eora_ix(); break;
				case 0xa9: adca_ix(); break;
				case 0xaa: ora_ix(); break;
				case 0xab: adda_ix(); break;
				case 0xac: cmpx_ix(); break;
				case 0xad: jsr_ix(); break;
				case 0xae: lds_ix(); break;
				case 0xaf: sts_ix(); break;
				case 0xb0: suba_ex(); break;
				case 0xb1: cmpa_ex(); break;
				case 0xb2: sbca_ex(); break;
				case 0xb3: illegal(); break;
				case 0xb4: anda_ex(); break;
				case 0xb5: bita_ex(); break;
				case 0xb6: lda_ex(); break;
				case 0xb7: sta_ex(); break;
				case 0xb8: eora_ex(); break;
				case 0xb9: adca_ex(); break;
				case 0xba: ora_ex(); break;
				case 0xbb: adda_ex(); break;
				case 0xbc: cmpx_ex(); break;
				case 0xbd: jsr_ex(); break;
				case 0xbe: lds_ex(); break;
				case 0xbf: sts_ex(); break;
				case 0xc0: subb_im(); break;
				case 0xc1: cmpb_im(); break;
				case 0xc2: sbcb_im(); break;
				case 0xc3: illegal(); break;
				case 0xc4: andb_im(); break;
				case 0xc5: bitb_im(); break;
				case 0xc6: ldb_im(); break;
				case 0xc7: stb_im(); break;
				case 0xc8: eorb_im(); break;
				case 0xc9: adcb_im(); break;
				case 0xca: orb_im(); break;
				case 0xcb: addb_im(); break;
				case 0xcc: illegal(); break;
				case 0xcd: illegal(); break;
				case 0xce: ldx_im(); break;
				case 0xcf: stx_im(); break;
				case 0xd0: subb_di(); break;
				case 0xd1: cmpb_di(); break;
				case 0xd2: sbcb_di(); break;
				case 0xd3: illegal(); break;
				case 0xd4: andb_di(); break;
				case 0xd5: bitb_di(); break;
				case 0xd6: ldb_di(); break;
				case 0xd7: stb_di(); break;
				case 0xd8: eorb_di(); break;
				case 0xd9: adcb_di(); break;
				case 0xda: orb_di(); break;
				case 0xdb: addb_di(); break;
				case 0xdc: illegal(); break;
				case 0xdd: illegal(); break;
				case 0xde: ldx_di(); break;
				case 0xdf: stx_di(); break;
				case 0xe0: subb_ix(); break;
				case 0xe1: cmpb_ix(); break;
				case 0xe2: sbcb_ix(); break;
				case 0xe3: illegal(); break;
				case 0xe4: andb_ix(); break;
				case 0xe5: bitb_ix(); break;
				case 0xe6: ldb_ix(); break;
				case 0xe7: stb_ix(); break;
				case 0xe8: eorb_ix(); break;
				case 0xe9: adcb_ix(); break;
				case 0xea: orb_ix(); break;
				case 0xeb: addb_ix(); break;
				case 0xec: illegal(); break;
				case 0xed: illegal(); break;
				case 0xee: ldx_ix(); break;
				case 0xef: stx_ix(); break;
				case 0xf0: subb_ex(); break;
				case 0xf1: cmpb_ex(); break;
				case 0xf2: sbcb_ex(); break;
				case 0xf3: illegal(); break;
				case 0xf4: andb_ex(); break;
				case 0xf5: bitb_ex(); break;
				case 0xf6: ldb_ex(); break;
				case 0xf7: stb_ex(); break;
				case 0xf8: eorb_ex(); break;
				case 0xf9: adcb_ex(); break;
				case 0xfa: orb_ex(); break;
				case 0xfb: addb_ex(); break;
				case 0xfc: addx_ex(); break;
				case 0xfd: illegal(); break;
				case 0xfe: ldx_ex(); break;
				case 0xff: stx_ex(); break;
			}
			INCREMENT_COUNTER(cycles_6800[ireg]);
		}
	} while( m6800_ICount>0 );

	INCREMENT_COUNTER(m6800.extra_cycles);
	m6800.extra_cycles = 0;

	return cycles - m6800_ICount;
}

/****************************************************************************
 * M6801 almost (fully?) equal to the M6803
 ****************************************************************************/
#if (HAS_M6801)
void m6801_init()
{
//  m6800.subtype = SUBTYPE_M6801;
	m6800.insn = m6803_insn;
	m6800.cycles = cycles_6803;
//	m6800.irq_callback = irqcallback;
//	state_register("m6801", index);
}
#endif

/****************************************************************************
 * M6802 almost (fully?) equal to the M6800
 ****************************************************************************/
#if (HAS_M6802)
static void m6802_init(int index, int clock, const void *config, int (*irqcallback)(int))
{
//  m6800.subtype   = SUBTYPE_M6802;
	m6800.insn = m6800_insn;
	m6800.cycles = cycles_6800;
	m6800.irq_callback = irqcallback;
	state_register("m6802", index);
}
#endif

/****************************************************************************
 * M6803 almost (fully?) equal to the M6801
 ****************************************************************************/
#if (HAS_M6803)
void m6803_init()
{
//  m6800.subtype = SUBTYPE_M6803;
	m6800.insn = m6803_insn;
	m6800.cycles = cycles_6803;
//	m6800.irq_callback = irqcallback;
//	state_register("m6803", index);
}
#endif

/****************************************************************************
 * Execute cycles CPU cycles. Return number of cycles really executed
 ****************************************************************************/
#if (HAS_M6803||HAS_M6801)
int m6803_execute(int cycles)
{
	UINT8 ireg;
	m6800_ICount = cycles;

	CLEANUP_conters;
	INCREMENT_COUNTER(m6803.extra_cycles);
	m6803.extra_cycles = 0;

	do
	{
		if( m6803.wai_state & M6800_WAI )
		{
			EAT_CYCLES;
		}
		else
		{
			pPPC = pPC;
//			CALL_MAME_DEBUG;
			ireg=M_RDOP(PCD);
			PC++;

			switch( ireg )
			{
				case 0x00: illegal(); break;
				case 0x01: nop(); break;
				case 0x02: illegal(); break;
				case 0x03: illegal(); break;
				case 0x04: lsrd(); /* 6803 only */; break;
				case 0x05: asld(); /* 6803 only */; break;
				case 0x06: tap(); break;
				case 0x07: tpa(); break;
				case 0x08: inx(); break;
				case 0x09: dex(); break;
				case 0x0A: CLV; break;
				case 0x0B: SEV; break;
				case 0x0C: CLC; break;
				case 0x0D: SEC; break;
				case 0x0E: cli(); break;
				case 0x0F: sei(); break;
				case 0x10: sba(); break;
				case 0x11: cba(); break;
				case 0x12: illegal(); break;
				case 0x13: illegal(); break;
				case 0x14: illegal(); break;
				case 0x15: illegal(); break;
				case 0x16: tab(); break;
				case 0x17: tba(); break;
				case 0x18: illegal(); break;
				case 0x19: daa(); break;
				case 0x1a: illegal(); break;
				case 0x1b: aba(); break;
				case 0x1c: illegal(); break;
				case 0x1d: illegal(); break;
				case 0x1e: illegal(); break;
				case 0x1f: illegal(); break;
				case 0x20: bra(); break;
				case 0x21: brn(); break;
				case 0x22: bhi(); break;
				case 0x23: bls(); break;
				case 0x24: bcc(); break;
				case 0x25: bcs(); break;
				case 0x26: bne(); break;
				case 0x27: beq(); break;
				case 0x28: bvc(); break;
				case 0x29: bvs(); break;
				case 0x2a: bpl(); break;
				case 0x2b: bmi(); break;
				case 0x2c: bge(); break;
				case 0x2d: blt(); break;
				case 0x2e: bgt(); break;
				case 0x2f: ble(); break;
				case 0x30: tsx(); break;
				case 0x31: ins(); break;
				case 0x32: pula(); break;
				case 0x33: pulb(); break;
				case 0x34: des(); break;
				case 0x35: txs(); break;
				case 0x36: psha(); break;
				case 0x37: pshb(); break;
				case 0x38: pulx(); /* 6803 only */ break;
				case 0x39: rts(); break;
				case 0x3a: abx(); /* 6803 only */ break;
				case 0x3b: rti(); break;
				case 0x3c: pshx(); /* 6803 only */ break;
				case 0x3d: mul(); /* 6803 only */ break;
				case 0x3e: wai(); break;
				case 0x3f: swi(); break;
				case 0x40: nega(); break;
				case 0x41: illegal(); break;
				case 0x42: illegal(); break;
				case 0x43: coma(); break;
				case 0x44: lsra(); break;
				case 0x45: illegal(); break;
				case 0x46: rora(); break;
				case 0x47: asra(); break;
				case 0x48: asla(); break;
				case 0x49: rola(); break;
				case 0x4a: deca(); break;
				case 0x4b: illegal(); break;
				case 0x4c: inca(); break;
				case 0x4d: tsta(); break;
				case 0x4e: illegal(); break;
				case 0x4f: clra(); break;
				case 0x50: negb(); break;
				case 0x51: illegal(); break;
				case 0x52: illegal(); break;
				case 0x53: comb(); break;
				case 0x54: lsrb(); break;
				case 0x55: illegal(); break;
				case 0x56: rorb(); break;
				case 0x57: asrb(); break;
				case 0x58: aslb(); break;
				case 0x59: rolb(); break;
				case 0x5a: decb(); break;
				case 0x5b: illegal(); break;
				case 0x5c: incb(); break;
				case 0x5d: tstb(); break;
				case 0x5e: illegal(); break;
				case 0x5f: clrb(); break;
				case 0x60: neg_ix(); break;
				case 0x61: illegal(); break;
				case 0x62: illegal(); break;
				case 0x63: com_ix(); break;
				case 0x64: lsr_ix(); break;
				case 0x65: illegal(); break;
				case 0x66: ror_ix(); break;
				case 0x67: asr_ix(); break;
				case 0x68: asl_ix(); break;
				case 0x69: rol_ix(); break;
				case 0x6a: dec_ix(); break;
				case 0x6b: illegal(); break;
				case 0x6c: inc_ix(); break;
				case 0x6d: tst_ix(); break;
				case 0x6e: jmp_ix(); break;
				case 0x6f: clr_ix(); break;
				case 0x70: neg_ex(); break;
				case 0x71: illegal(); break;
				case 0x72: illegal(); break;
				case 0x73: com_ex(); break;
				case 0x74: lsr_ex(); break;
				case 0x75: illegal(); break;
				case 0x76: ror_ex(); break;
				case 0x77: asr_ex(); break;
				case 0x78: asl_ex(); break;
				case 0x79: rol_ex(); break;
				case 0x7a: dec_ex(); break;
				case 0x7b: illegal(); break;
				case 0x7c: inc_ex(); break;
				case 0x7d: tst_ex(); break;
				case 0x7e: jmp_ex(); break;
				case 0x7f: clr_ex(); break;
				case 0x80: suba_im(); break;
				case 0x81: cmpa_im(); break;
				case 0x82: sbca_im(); break;
				case 0x83: subd_im(); /* 6803 only */ break;
				case 0x84: anda_im(); break;
				case 0x85: bita_im(); break;
				case 0x86: lda_im(); break;
				case 0x87: sta_im(); break;
				case 0x88: eora_im(); break;
				case 0x89: adca_im(); break;
				case 0x8a: ora_im(); break;
				case 0x8b: adda_im(); break;
				case 0x8c: cpx_im(); /* 6803 difference */ break;
				case 0x8d: bsr(); break;
				case 0x8e: lds_im(); break;
				case 0x8f: sts_im(); /* orthogonality */ break;
				case 0x90: suba_di(); break;
				case 0x91: cmpa_di(); break;
				case 0x92: sbca_di(); break;
				case 0x93: subd_di(); /* 6803 only */ break;
				case 0x94: anda_di(); break;
				case 0x95: bita_di(); break;
				case 0x96: lda_di(); break;
				case 0x97: sta_di(); break;
				case 0x98: eora_di(); break;
				case 0x99: adca_di(); break;
				case 0x9a: ora_di(); break;
				case 0x9b: adda_di(); break;
				case 0x9c: cpx_di(); /* 6803 difference */ break;
				case 0x9d: jsr_di(); break;
				case 0x9e: lds_di(); break;
				case 0x9f: sts_di(); break;
				case 0xa0: suba_ix(); break;
				case 0xa1: cmpa_ix(); break;
				case 0xa2: sbca_ix(); break;
				case 0xa3: subd_ix(); /* 6803 only */ break;
				case 0xa4: anda_ix(); break;
				case 0xa5: bita_ix(); break;
				case 0xa6: lda_ix(); break;
				case 0xa7: sta_ix(); break;
				case 0xa8: eora_ix(); break;
				case 0xa9: adca_ix(); break;
				case 0xaa: ora_ix(); break;
				case 0xab: adda_ix(); break;
				case 0xac: cpx_ix(); /* 6803 difference */ break;
				case 0xad: jsr_ix(); break;
				case 0xae: lds_ix(); break;
				case 0xaf: sts_ix(); break;
				case 0xb0: suba_ex(); break;
				case 0xb1: cmpa_ex(); break;
				case 0xb2: sbca_ex(); break;
				case 0xb3: subd_ex(); /* 6803 only */ break;
				case 0xb4: anda_ex(); break;
				case 0xb5: bita_ex(); break;
				case 0xb6: lda_ex(); break;
				case 0xb7: sta_ex(); break;
				case 0xb8: eora_ex(); break;
				case 0xb9: adca_ex(); break;
				case 0xba: ora_ex(); break;
				case 0xbb: adda_ex(); break;
				case 0xbc: cpx_ex(); /* 6803 difference */ break;
				case 0xbd: jsr_ex(); break;
				case 0xbe: lds_ex(); break;
				case 0xbf: sts_ex(); break;
				case 0xc0: subb_im(); break;
				case 0xc1: cmpb_im(); break;
				case 0xc2: sbcb_im(); break;
				case 0xc3: addd_im(); /* 6803 only */ break;
				case 0xc4: andb_im(); break;
				case 0xc5: bitb_im(); break;
				case 0xc6: ldb_im(); break;
				case 0xc7: stb_im(); break;
				case 0xc8: eorb_im(); break;
				case 0xc9: adcb_im(); break;
				case 0xca: orb_im(); break;
				case 0xcb: addb_im(); break;
				case 0xcc: ldd_im(); /* 6803 only */ break;
				case 0xcd: std_im(); /* 6803 only -- orthogonality */ break;
				case 0xce: ldx_im(); break;
				case 0xcf: stx_im(); break;
				case 0xd0: subb_di(); break;
				case 0xd1: cmpb_di(); break;
				case 0xd2: sbcb_di(); break;
				case 0xd3: addd_di(); /* 6803 only */ break;
				case 0xd4: andb_di(); break;
				case 0xd5: bitb_di(); break;
				case 0xd6: ldb_di(); break;
				case 0xd7: stb_di(); break;
				case 0xd8: eorb_di(); break;
				case 0xd9: adcb_di(); break;
				case 0xda: orb_di(); break;
				case 0xdb: addb_di(); break;
				case 0xdc: ldd_di(); /* 6803 only */ break;
				case 0xdd: std_di(); /* 6803 only */ break;
				case 0xde: ldx_di(); break;
				case 0xdf: stx_di(); break;
				case 0xe0: subb_ix(); break;
				case 0xe1: cmpb_ix(); break;
				case 0xe2: sbcb_ix(); break;
				case 0xe3: addd_ix(); /* 6803 only */ break;
				case 0xe4: andb_ix(); break;
				case 0xe5: bitb_ix(); break;
				case 0xe6: ldb_ix(); break;
				case 0xe7: stb_ix(); break;
				case 0xe8: eorb_ix(); break;
				case 0xe9: adcb_ix(); break;
				case 0xea: orb_ix(); break;
				case 0xeb: addb_ix(); break;
				case 0xec: ldd_ix(); /* 6803 only */ break;
				case 0xed: std_ix(); /* 6803 only */ break;
				case 0xee: ldx_ix(); break;
				case 0xef: stx_ix(); break;
				case 0xf0: subb_ex(); break;
				case 0xf1: cmpb_ex(); break;
				case 0xf2: sbcb_ex(); break;
				case 0xf3: addd_ex(); /* 6803 only */ break;
				case 0xf4: andb_ex(); break;
				case 0xf5: bitb_ex(); break;
				case 0xf6: ldb_ex(); break;
				case 0xf7: stb_ex(); break;
				case 0xf8: eorb_ex(); break;
				case 0xf9: adcb_ex(); break;
				case 0xfa: orb_ex(); break;
				case 0xfb: addb_ex(); break;
				case 0xfc: ldd_ex(); /* 6803 only */ break;
				case 0xfd: std_ex(); /* 6803 only */ break;
				case 0xfe: ldx_ex(); break;
				case 0xff: stx_ex(); break;
			}
			INCREMENT_COUNTER(cycles_6803[ireg]);
		}
	} while( m6800_ICount>0 );

	INCREMENT_COUNTER(m6803.extra_cycles);
	m6803.extra_cycles = 0;

	return cycles - m6800_ICount;
}
#endif

#if (HAS_M6803)

//static READ8_HANDLER( m6803_internal_registers_r );
//static WRITE8_HANDLER( m6803_internal_registers_w );

//static ADDRESS_MAP_START(m6803_mem, ADDRESS_SPACE_PROGRAM, 8)
//	AM_RANGE(0x0000, 0x001f) AM_READWRITE(m6803_internal_registers_r, m6803_internal_registers_w)
//	AM_RANGE(0x0020, 0x007f) AM_NOP        /* unused */
//	AM_RANGE(0x0080, 0x00ff) AM_RAM        /* 6803 internal RAM */
//ADDRESS_MAP_END

#endif

/****************************************************************************
 * M6808 almost (fully?) equal to the M6800
 ****************************************************************************/
#if (HAS_M6808)
static void m6808_init(int index, int clock, const void *config, int (*irqcallback)(int))
{
//  m6800.subtype = SUBTYPE_M6808;
	m6800.insn = m6800_insn;
	m6800.cycles = cycles_6800;
	m6800.irq_callback = irqcallback;
	state_register("m6808", index);
}
#endif

/****************************************************************************
 * HD63701 similiar to the M6800
 ****************************************************************************/
#if (HAS_HD63701)
void hd63701_init()
{
//  m6800.subtype = SUBTYPE_HD63701;
	m6800.insn = hd63701_insn;
	m6800.cycles = cycles_63701;
//	m6800.irq_callback = irqcallback;
//	state_register("hd63701", index);
}
/****************************************************************************
 * Execute cycles CPU cycles. Return number of cycles really executed
 ****************************************************************************/
int hd63701_execute(int cycles)
{
	UINT8 ireg;
	m6800_ICount = cycles;

	CLEANUP_conters;
	INCREMENT_COUNTER(hd63701.extra_cycles);
	hd63701.extra_cycles = 0;

	do
	{
		if( hd63701.wai_state & (HD63701_WAI|HD63701_SLP) )
		{
			EAT_CYCLES;
		}
		else
		{
			pPPC = pPC;
//			CALL_MAME_DEBUG;
			ireg=M_RDOP(PCD);
			PC++;

			switch( ireg )
			{
				case 0x00: trap(); break;
				case 0x01: nop(); break;
				case 0x02: trap(); break;
				case 0x03: trap(); break;
				case 0x04: lsrd(); /* 6803 only */; break;
				case 0x05: asld(); /* 6803 only */; break;
				case 0x06: tap(); break;
				case 0x07: tpa(); break;
				case 0x08: inx(); break;
				case 0x09: dex(); break;
				case 0x0A: CLV; break;
				case 0x0B: SEV; break;
				case 0x0C: CLC; break;
				case 0x0D: SEC; break;
				case 0x0E: cli(); break;
				case 0x0F: sei(); break;
				case 0x10: sba(); break;
				case 0x11: cba(); break;
				case 0x12: undoc1(); break;
				case 0x13: undoc2(); break;
				case 0x14: trap(); break;
				case 0x15: trap(); break;
				case 0x16: tab(); break;
				case 0x17: tba(); break;
				case 0x18: xgdx(); /* HD63701YO only */; break;
				case 0x19: daa(); break;
				case 0x1a: slp(); break;
				case 0x1b: aba(); break;
				case 0x1c: trap(); break;
				case 0x1d: trap(); break;
				case 0x1e: trap(); break;
				case 0x1f: trap(); break;
				case 0x20: bra(); break;
				case 0x21: brn(); break;
				case 0x22: bhi(); break;
				case 0x23: bls(); break;
				case 0x24: bcc(); break;
				case 0x25: bcs(); break;
				case 0x26: bne(); break;
				case 0x27: beq(); break;
				case 0x28: bvc(); break;
				case 0x29: bvs(); break;
				case 0x2a: bpl(); break;
				case 0x2b: bmi(); break;
				case 0x2c: bge(); break;
				case 0x2d: blt(); break;
				case 0x2e: bgt(); break;
				case 0x2f: ble(); break;
				case 0x30: tsx(); break;
				case 0x31: ins(); break;
				case 0x32: pula(); break;
				case 0x33: pulb(); break;
				case 0x34: des(); break;
				case 0x35: txs(); break;
				case 0x36: psha(); break;
				case 0x37: pshb(); break;
				case 0x38: pulx(); /* 6803 only */ break;
				case 0x39: rts(); break;
				case 0x3a: abx(); /* 6803 only */ break;
				case 0x3b: rti(); break;
				case 0x3c: pshx(); /* 6803 only */ break;
				case 0x3d: mul(); /* 6803 only */ break;
				case 0x3e: wai(); break;
				case 0x3f: swi(); break;
				case 0x40: nega(); break;
				case 0x41: trap(); break;
				case 0x42: trap(); break;
				case 0x43: coma(); break;
				case 0x44: lsra(); break;
				case 0x45: trap(); break;
				case 0x46: rora(); break;
				case 0x47: asra(); break;
				case 0x48: asla(); break;
				case 0x49: rola(); break;
				case 0x4a: deca(); break;
				case 0x4b: trap(); break;
				case 0x4c: inca(); break;
				case 0x4d: tsta(); break;
				case 0x4e: trap(); break;
				case 0x4f: clra(); break;
				case 0x50: negb(); break;
				case 0x51: trap(); break;
				case 0x52: trap(); break;
				case 0x53: comb(); break;
				case 0x54: lsrb(); break;
				case 0x55: trap(); break;
				case 0x56: rorb(); break;
				case 0x57: asrb(); break;
				case 0x58: aslb(); break;
				case 0x59: rolb(); break;
				case 0x5a: decb(); break;
				case 0x5b: trap(); break;
				case 0x5c: incb(); break;
				case 0x5d: tstb(); break;
				case 0x5e: trap(); break;
				case 0x5f: clrb(); break;
				case 0x60: neg_ix(); break;
				case 0x61: aim_ix(); /* HD63701YO only */; break;
				case 0x62: oim_ix(); /* HD63701YO only */; break;
				case 0x63: com_ix(); break;
				case 0x64: lsr_ix(); break;
				case 0x65: eim_ix(); /* HD63701YO only */; break;
				case 0x66: ror_ix(); break;
				case 0x67: asr_ix(); break;
				case 0x68: asl_ix(); break;
				case 0x69: rol_ix(); break;
				case 0x6a: dec_ix(); break;
				case 0x6b: tim_ix(); /* HD63701YO only */; break;
				case 0x6c: inc_ix(); break;
				case 0x6d: tst_ix(); break;
				case 0x6e: jmp_ix(); break;
				case 0x6f: clr_ix(); break;
				case 0x70: neg_ex(); break;
				case 0x71: aim_di(); /* HD63701YO only */; break;
				case 0x72: oim_di(); /* HD63701YO only */; break;
				case 0x73: com_ex(); break;
				case 0x74: lsr_ex(); break;
				case 0x75: eim_di(); /* HD63701YO only */; break;
				case 0x76: ror_ex(); break;
				case 0x77: asr_ex(); break;
				case 0x78: asl_ex(); break;
				case 0x79: rol_ex(); break;
				case 0x7a: dec_ex(); break;
				case 0x7b: tim_di(); /* HD63701YO only */; break;
				case 0x7c: inc_ex(); break;
				case 0x7d: tst_ex(); break;
				case 0x7e: jmp_ex(); break;
				case 0x7f: clr_ex(); break;
				case 0x80: suba_im(); break;
				case 0x81: cmpa_im(); break;
				case 0x82: sbca_im(); break;
				case 0x83: subd_im(); /* 6803 only */ break;
				case 0x84: anda_im(); break;
				case 0x85: bita_im(); break;
				case 0x86: lda_im(); break;
				case 0x87: sta_im(); break;
				case 0x88: eora_im(); break;
				case 0x89: adca_im(); break;
				case 0x8a: ora_im(); break;
				case 0x8b: adda_im(); break;
				case 0x8c: cpx_im(); /* 6803 difference */ break;
				case 0x8d: bsr(); break;
				case 0x8e: lds_im(); break;
				case 0x8f: sts_im(); /* orthogonality */ break;
				case 0x90: suba_di(); break;
				case 0x91: cmpa_di(); break;
				case 0x92: sbca_di(); break;
				case 0x93: subd_di(); /* 6803 only */ break;
				case 0x94: anda_di(); break;
				case 0x95: bita_di(); break;
				case 0x96: lda_di(); break;
				case 0x97: sta_di(); break;
				case 0x98: eora_di(); break;
				case 0x99: adca_di(); break;
				case 0x9a: ora_di(); break;
				case 0x9b: adda_di(); break;
				case 0x9c: cpx_di(); /* 6803 difference */ break;
				case 0x9d: jsr_di(); break;
				case 0x9e: lds_di(); break;
				case 0x9f: sts_di(); break;
				case 0xa0: suba_ix(); break;
				case 0xa1: cmpa_ix(); break;
				case 0xa2: sbca_ix(); break;
				case 0xa3: subd_ix(); /* 6803 only */ break;
				case 0xa4: anda_ix(); break;
				case 0xa5: bita_ix(); break;
				case 0xa6: lda_ix(); break;
				case 0xa7: sta_ix(); break;
				case 0xa8: eora_ix(); break;
				case 0xa9: adca_ix(); break;
				case 0xaa: ora_ix(); break;
				case 0xab: adda_ix(); break;
				case 0xac: cpx_ix(); /* 6803 difference */ break;
				case 0xad: jsr_ix(); break;
				case 0xae: lds_ix(); break;
				case 0xaf: sts_ix(); break;
				case 0xb0: suba_ex(); break;
				case 0xb1: cmpa_ex(); break;
				case 0xb2: sbca_ex(); break;
				case 0xb3: subd_ex(); /* 6803 only */ break;
				case 0xb4: anda_ex(); break;
				case 0xb5: bita_ex(); break;
				case 0xb6: lda_ex(); break;
				case 0xb7: sta_ex(); break;
				case 0xb8: eora_ex(); break;
				case 0xb9: adca_ex(); break;
				case 0xba: ora_ex(); break;
				case 0xbb: adda_ex(); break;
				case 0xbc: cpx_ex(); /* 6803 difference */ break;
				case 0xbd: jsr_ex(); break;
				case 0xbe: lds_ex(); break;
				case 0xbf: sts_ex(); break;
				case 0xc0: subb_im(); break;
				case 0xc1: cmpb_im(); break;
				case 0xc2: sbcb_im(); break;
				case 0xc3: addd_im(); /* 6803 only */ break;
				case 0xc4: andb_im(); break;
				case 0xc5: bitb_im(); break;
				case 0xc6: ldb_im(); break;
				case 0xc7: stb_im(); break;
				case 0xc8: eorb_im(); break;
				case 0xc9: adcb_im(); break;
				case 0xca: orb_im(); break;
				case 0xcb: addb_im(); break;
				case 0xcc: ldd_im(); /* 6803 only */ break;
				case 0xcd: std_im(); /* 6803 only -- orthogonality */ break;
				case 0xce: ldx_im(); break;
				case 0xcf: stx_im(); break;
				case 0xd0: subb_di(); break;
				case 0xd1: cmpb_di(); break;
				case 0xd2: sbcb_di(); break;
				case 0xd3: addd_di(); /* 6803 only */ break;
				case 0xd4: andb_di(); break;
				case 0xd5: bitb_di(); break;
				case 0xd6: ldb_di(); break;
				case 0xd7: stb_di(); break;
				case 0xd8: eorb_di(); break;
				case 0xd9: adcb_di(); break;
				case 0xda: orb_di(); break;
				case 0xdb: addb_di(); break;
				case 0xdc: ldd_di(); /* 6803 only */ break;
				case 0xdd: std_di(); /* 6803 only */ break;
				case 0xde: ldx_di(); break;
				case 0xdf: stx_di(); break;
				case 0xe0: subb_ix(); break;
				case 0xe1: cmpb_ix(); break;
				case 0xe2: sbcb_ix(); break;
				case 0xe3: addd_ix(); /* 6803 only */ break;
				case 0xe4: andb_ix(); break;
				case 0xe5: bitb_ix(); break;
				case 0xe6: ldb_ix(); break;
				case 0xe7: stb_ix(); break;
				case 0xe8: eorb_ix(); break;
				case 0xe9: adcb_ix(); break;
				case 0xea: orb_ix(); break;
				case 0xeb: addb_ix(); break;
				case 0xec: ldd_ix(); /* 6803 only */ break;
				case 0xed: std_ix(); /* 6803 only */ break;
				case 0xee: ldx_ix(); break;
				case 0xef: stx_ix(); break;
				case 0xf0: subb_ex(); break;
				case 0xf1: cmpb_ex(); break;
				case 0xf2: sbcb_ex(); break;
				case 0xf3: addd_ex(); /* 6803 only */ break;
				case 0xf4: andb_ex(); break;
				case 0xf5: bitb_ex(); break;
				case 0xf6: ldb_ex(); break;
				case 0xf7: stb_ex(); break;
				case 0xf8: eorb_ex(); break;
				case 0xf9: adcb_ex(); break;
				case 0xfa: orb_ex(); break;
				case 0xfb: addb_ex(); break;
				case 0xfc: ldd_ex(); /* 6803 only */ break;
				case 0xfd: std_ex(); /* 6803 only */ break;
				case 0xfe: ldx_ex(); break;
				case 0xff: stx_ex(); break;
			}
			INCREMENT_COUNTER(cycles_63701[ireg]);
		}
	} while( m6800_ICount>0 );

	INCREMENT_COUNTER(hd63701.extra_cycles);
	hd63701.extra_cycles = 0;

	return cycles - m6800_ICount;
}

/*
    if change_pc() direccted these areas ,Call hd63701_trap_pc().
    'mode' is selected by the sense of p2.0,p2.1,and p2.3 at reset timming.
    mode 0,1,2,4,6 : $0000-$001f
    mode 5         : $0000-$001f,$0200-$efff
    mode 7         : $0000-$001f,$0100-$efff
*/
void hd63701_trap_pc(void)
{
	TAKE_TRAP;
}

//static READ8_HANDLER( m6803_internal_registers_r );
//static WRITE8_HANDLER( m6803_internal_registers_w );

//READ8_HANDLER( hd63701_internal_registers_r )
//{
//	return m6803_internal_registers_r(offset);
//}

//WRITE8_HANDLER( hd63701_internal_registers_w )
//{
//	m6803_internal_registers_w(offset,data);
//}
#endif

/****************************************************************************
 * NSC-8105 similiar to the M6800, but the opcodes are scrambled and there
 * is at least one new opcode ($fc)
 ****************************************************************************/
#if (HAS_NSC8105)
static void nsc8105_init(int index, int clock, const void *config, int (*irqcallback)(int))
{
//  m6800.subtype = SUBTYPE_NSC8105;
	m6800.insn = nsc8105_insn;
	m6800.cycles = cycles_nsc8105;
	state_register("nsc8105", index);
}
/****************************************************************************
 * Execute cycles CPU cycles. Return number of cycles really executed
 ****************************************************************************/
static int nsc8105_execute(int cycles)
{
	UINT8 ireg;
	m6800_ICount = cycles;

	CLEANUP_conters;
	INCREMENT_COUNTER(nsc8105.extra_cycles);
	nsc8105.extra_cycles = 0;

	do
	{
		if( nsc8105.wai_state & NSC8105_WAI )
		{
			EAT_CYCLES;
		}
		else
		{
			pPPC = pPC;
			CALL_MAME_DEBUG;
			ireg=M_RDOP(PCD);
			PC++;

			switch( ireg )
			{
				case 0x00: illegal(); break;
				case 0x01: illegal(); break;
				case 0x02: nop(); break;
				case 0x03: illegal(); break;
				case 0x04: illegal(); break;
				case 0x05: tap(); break;
				case 0x06: illegal(); break;
				case 0x07: tpa(); break;
				case 0x08: inx(); break;
				case 0x09: CLV; break;
				case 0x0a: dex(); break;
				case 0x0b: SEV; break;
				case 0x0c: CLC; break;
				case 0x0d: cli(); break;
				case 0x0e: SEC; break;
				case 0x0f: sei(); break;
				case 0x10: sba(); break;
				case 0x11: illegal(); break;
				case 0x12: cba(); break;
				case 0x13: illegal(); break;
				case 0x14: illegal(); break;
				case 0x15: tab(); break;
				case 0x16: illegal(); break;
				case 0x17: tba(); break;
				case 0x18: illegal(); break;
				case 0x19: illegal(); break;
				case 0x1a: daa(); break;
				case 0x1b: aba(); break;
				case 0x1c: illegal(); break;
				case 0x1d: illegal(); break;
				case 0x1e: illegal(); break;
				case 0x1f: illegal(); break;
				case 0x20: bra(); break;
				case 0x21: bhi(); break;
				case 0x22: brn(); break;
				case 0x23: bls(); break;
				case 0x24: bcc(); break;
				case 0x25: bne(); break;
				case 0x26: bcs(); break;
				case 0x27: beq(); break;
				case 0x28: bvc(); break;
				case 0x29: bpl(); break;
				case 0x2a: bvs(); break;
				case 0x2b: bmi(); break;
				case 0x2c: bge(); break;
				case 0x2d: bgt(); break;
				case 0x2e: blt(); break;
				case 0x2f: ble(); break;
				case 0x30: tsx(); break;
				case 0x31: pula(); break;
				case 0x32: ins(); break;
				case 0x33: pulb(); break;
				case 0x34: des(); break;
				case 0x35: psha(); break;
				case 0x36: txs(); break;
				case 0x37: pshb(); break;
				case 0x38: illegal(); break;
				case 0x39: illegal(); break;
				case 0x3a: rts(); break;
				case 0x3b: rti(); break;
				case 0x3c: illegal(); break;
				case 0x3d: wai(); break;
				case 0x3e: illegal(); break;
				case 0x3f: swi(); break;
				case 0x40: suba_im(); break;
				case 0x41: sbca_im(); break;
				case 0x42: cmpa_im(); break;
				case 0x43: illegal(); break;
				case 0x44: anda_im(); break;
				case 0x45: lda_im(); break;
				case 0x46: bita_im(); break;
				case 0x47: sta_im(); break;
				case 0x48: eora_im(); break;
				case 0x49: ora_im(); break;
				case 0x4a: adca_im(); break;
				case 0x4b: adda_im(); break;
				case 0x4c: cmpx_im(); break;
				case 0x4d: lds_im(); break;
				case 0x4e: bsr(); break;
				case 0x4f: sts_im(); /* orthogonality */ break;
				case 0x50: suba_di(); break;
				case 0x51: sbca_di(); break;
				case 0x52: cmpa_di(); break;
				case 0x53: illegal(); break;
				case 0x54: anda_di(); break;
				case 0x55: lda_di(); break;
				case 0x56: bita_di(); break;
				case 0x57: sta_di(); break;
				case 0x58: eora_di(); break;
				case 0x59: ora_di(); break;
				case 0x5a: adca_di(); break;
				case 0x5b: adda_di(); break;
				case 0x5c: cmpx_di(); break;
				case 0x5d: lds_di(); break;
				case 0x5e: jsr_di(); break;
				case 0x5f: sts_di(); break;
				case 0x60: suba_ix(); break;
				case 0x61: sbca_ix(); break;
				case 0x62: cmpa_ix(); break;
				case 0x63: illegal(); break;
				case 0x64: anda_ix(); break;
				case 0x65: lda_ix(); break;
				case 0x66: bita_ix(); break;
				case 0x67: sta_ix(); break;
				case 0x68: eora_ix(); break;
				case 0x69: ora_ix(); break;
				case 0x6a: adca_ix(); break;
				case 0x6b: adda_ix(); break;
				case 0x6c: cmpx_ix(); break;
				case 0x6d: lds_ix(); break;
				case 0x6e: jsr_ix(); break;
				case 0x6f: sts_ix(); break;
				case 0x70: suba_ex(); break;
				case 0x71: sbca_ex(); break;
				case 0x72: cmpa_ex(); break;
				case 0x73: illegal(); break;
				case 0x74: anda_ex(); break;
				case 0x75: lda_ex(); break;
				case 0x76: bita_ex(); break;
				case 0x77: sta_ex(); break;
				case 0x78: eora_ex(); break;
				case 0x79: ora_ex(); break;
				case 0x7a: adca_ex(); break;
				case 0x7b: adda_ex(); break;
				case 0x7c: cmpx_ex(); break;
				case 0x7d: lds_ex(); break;
				case 0x7e: jsr_ex(); break;
				case 0x7f: sts_ex(); break;
				case 0x80: nega(); break;
				case 0x81: illegal(); break;
				case 0x82: illegal(); break;
				case 0x83: coma(); break;
				case 0x84: lsra(); break;
				case 0x85: rora(); break;
				case 0x86: illegal(); break;
				case 0x87: asra(); break;
				case 0x88: asla(); break;
				case 0x89: deca(); break;
				case 0x8a: rola(); break;
				case 0x8b: illegal(); break;
				case 0x8c: inca(); break;
				case 0x8d: illegal(); break;
				case 0x8e: tsta(); break;
				case 0x8f: clra(); break;
				case 0x90: negb(); break;
				case 0x91: illegal(); break;
				case 0x92: illegal(); break;
				case 0x93: comb(); break;
				case 0x94: lsrb(); break;
				case 0x95: rorb(); break;
				case 0x96: illegal(); break;
				case 0x97: asrb(); break;
				case 0x98: aslb(); break;
				case 0x99: decb(); break;
				case 0x9a: rolb(); break;
				case 0x9b: illegal(); break;
				case 0x9c: incb(); break;
				case 0x9d: illegal(); break;
				case 0x9e: tstb(); break;
				case 0x9f: clrb(); break;
				case 0xa0: neg_ix(); break;
				case 0xa1: illegal(); break;
				case 0xa2: illegal(); break;
				case 0xa3: com_ix(); break;
				case 0xa4: lsr_ix(); break;
				case 0xa5: ror_ix(); break;
				case 0xa6: illegal(); break;
				case 0xa7: asr_ix(); break;
				case 0xa8: asl_ix(); break;
				case 0xa9: dec_ix(); break;
				case 0xaa: rol_ix(); break;
				case 0xab: illegal(); break;
				case 0xac: inc_ix(); break;
				case 0xad: jmp_ix(); break;
				case 0xae: tst_ix(); break;
				case 0xaf: clr_ix(); break;
				case 0xb0: neg_ex(); break;
				case 0xb1: illegal(); break;
				case 0xb2: illegal(); break;
				case 0xb3: com_ex(); break;
				case 0xb4: lsr_ex(); break;
				case 0xb5: ror_ex(); break;
				case 0xb6: illegal(); break;
				case 0xb7: asr_ex(); break;
				case 0xb8: asl_ex(); break;
				case 0xb9: dec_ex(); break;
				case 0xba: rol_ex(); break;
				case 0xbb: illegal(); break;
				case 0xbc: inc_ex(); break;
				case 0xbd: jmp_ex(); break;
				case 0xbe: tst_ex(); break;
				case 0xbf: clr_ex(); break;
				case 0xc0: subb_im(); break;
				case 0xc1: sbcb_im(); break;
				case 0xc2: cmpb_im(); break;
				case 0xc3: illegal(); break;
				case 0xc4: andb_im(); break;
				case 0xc5: ldb_im(); break;
				case 0xc6: bitb_im(); break;
				case 0xc7: stb_im(); break;
				case 0xc8: eorb_im(); break;
				case 0xc9: orb_im(); break;
				case 0xca: adcb_im(); break;
				case 0xcb: addb_im(); break;
				case 0xcc: illegal(); break;
				case 0xcd: ldx_im(); break;
				case 0xce: illegal(); break;
				case 0xcf: stx_im(); break;
				case 0xd0: subb_di(); break;
				case 0xd1: sbcb_di(); break;
				case 0xd2: cmpb_di(); break;
				case 0xd3: illegal(); break;
				case 0xd4: andb_di(); break;
				case 0xd5: ldb_di(); break;
				case 0xd6: bitb_di(); break;
				case 0xd7: stb_di(); break;
				case 0xd8: eorb_di(); break;
				case 0xd9: orb_di(); break;
				case 0xda: adcb_di(); break;
				case 0xdb: addb_di(); break;
				case 0xdc: illegal(); break;
				case 0xdd: ldx_di(); break;
				case 0xde: illegal(); break;
				case 0xdf: stx_di(); break;
				case 0xe0: subb_ix(); break;
				case 0xe1: sbcb_ix(); break;
				case 0xe2: cmpb_ix(); break;
				case 0xe3: illegal(); break;
				case 0xe4: andb_ix(); break;
				case 0xe5: ldb_ix(); break;
				case 0xe6: bitb_ix(); break;
				case 0xe7: stb_ix(); break;
				case 0xe8: eorb_ix(); break;
				case 0xe9: orb_ix(); break;
				case 0xea: adcb_ix(); break;
				case 0xeb: addb_ix(); break;
				case 0xec: adcx_im(); break; /* NSC8105 only */
				case 0xed: ldx_ix(); break;
				case 0xee: illegal(); break;
				case 0xef: stx_ix(); break;
				case 0xf0: subb_ex(); break;
				case 0xf1: sbcb_ex(); break;
				case 0xf2: cmpb_ex(); break;
				case 0xf3: illegal(); break;
				case 0xf4: andb_ex(); break;
				case 0xf5: ldb_ex(); break;
				case 0xf6: bitb_ex(); break;
				case 0xf7: stb_ex(); break;
				case 0xf8: eorb_ex(); break;
				case 0xf9: orb_ex(); break;
				case 0xfa: adcb_ex(); break;
				case 0xfb: addb_ex(); break;
				case 0xfc: addx_ex(); break;
				case 0xfd: ldx_ex(); break;
				case 0xfe: illegal(); break;
				case 0xff: stx_ex(); break;
			}
			INCREMENT_COUNTER(cycles_nsc8105[ireg]);
		}
	} while( m6800_ICount>0 );

	INCREMENT_COUNTER(nsc8105.extra_cycles);
	nsc8105.extra_cycles = 0;

	return cycles - m6800_ICount;
}
#endif


#if (HAS_M6803||HAS_HD63701)

#if 0
static unsigned char m6803_internal_registers_r(unsigned short offset)
{
	switch (offset)
	{
		case 0x00:
			return m6800.port1_ddr;
		case 0x01:
			return m6800.port2_ddr;
		case 0x02:
			return (M6800_io_read_byte_8(M6803_PORT1) & (m6800.port1_ddr ^ 0xff))
					| (m6800.port1_data & m6800.port1_ddr);
		case 0x03:
			return (M6800_io_read_byte_8(M6803_PORT2) & (m6800.port2_ddr ^ 0xff))
					| (m6800.port2_data & m6800.port2_ddr);
		case 0x04:
			return m6800.port3_ddr;
		case 0x05:
			return m6800.port4_ddr;
		case 0x06:
			return (M6800_io_read_byte_8(M6803_PORT3) & (m6800.port3_ddr ^ 0xff))
					| (m6800.port3_data & m6800.port3_ddr);
		case 0x07:
			return (M6800_io_read_byte_8(M6803_PORT4) & (m6800.port4_ddr ^ 0xff))
					| (m6800.port4_data & m6800.port4_ddr);
		case 0x08:
			m6800.pending_tcsr = 0;
//logerror("CPU #%d PC %04x: warning - read TCSR register\n",cpu_getactivecpu(),activecpu_get_pc());
			return m6800.tcsr;
		case 0x09:
			if(!(m6800.pending_tcsr&TCSR_TOF))
			{
				m6800.tcsr &= ~TCSR_TOF;
				MODIFIED_tcsr;
			}
			return m6800.counter.b.h;
		case 0x0a:
			return m6800.counter.b.l;
		case 0x0b:
			if(!(m6800.pending_tcsr&TCSR_OCF))
			{
				m6800.tcsr &= ~TCSR_OCF;
				MODIFIED_tcsr;
			}
			return m6800.output_compare.b.h;
		case 0x0c:
			if(!(m6800.pending_tcsr&TCSR_OCF))
			{
				m6800.tcsr &= ~TCSR_OCF;
				MODIFIED_tcsr;
			}
			return m6800.output_compare.b.l;
		case 0x0d:
			if(!(m6800.pending_tcsr&TCSR_ICF))
			{
				m6800.tcsr &= ~TCSR_ICF;
				MODIFIED_tcsr;
			}
			return (m6800.input_capture >> 0) & 0xff;
		case 0x0e:
			return (m6800.input_capture >> 8) & 0xff;
		case 0x0f:
		case 0x10:
		case 0x11:
		case 0x12:
		case 0x13:
//			logerror("CPU #%d PC %04x: warning - read from unsupported internal register %02x\n",cpu_getactivecpu(),activecpu_get_pc(),offset);
			return 0;
		case 0x14:
//			logerror("CPU #%d PC %04x: read RAM control register\n",cpu_getactivecpu(),activecpu_get_pc());
			return m6800.ram_ctrl;
		case 0x15:
		case 0x16:
		case 0x17:
		case 0x18:
		case 0x19:
		case 0x1a:
		case 0x1b:
		case 0x1c:
		case 0x1d:
		case 0x1e:
		case 0x1f:
		default:
//			logerror("CPU #%d PC %04x: warning - read from reserved internal register %02x\n",cpu_getactivecpu(),activecpu_get_pc(),offset);
			return 0;
	}
}
#endif

void m6803_internal_registers_w(unsigned short offset, unsigned char data)
{
	static int latch09;

	switch (offset)
	{
		case 0x00:
			if (m6800.port1_ddr != data)
			{
				m6800.port1_ddr = data;
				if(m6800.port1_ddr == 0xff)
					M6800_io_write_byte_8(M6803_PORT1,m6800.port1_data);
				else
					M6800_io_write_byte_8(M6803_PORT1,(m6800.port1_data & m6800.port1_ddr)
						| (M6800_io_read_byte_8(M6803_PORT1) & (m6800.port1_ddr ^ 0xff)));
			}
			break;
		case 0x01:
			if (m6800.port2_ddr != data)
			{
				m6800.port2_ddr = data;
				if(m6800.port2_ddr == 0xff)
					M6800_io_write_byte_8(M6803_PORT2,m6800.port2_data);
				else
					M6800_io_write_byte_8(M6803_PORT2,(m6800.port2_data & m6800.port2_ddr)
						| (M6800_io_read_byte_8(M6803_PORT2) & (m6800.port2_ddr ^ 0xff)));

//				if (m6800.port2_ddr & 2)
//					logerror("CPU #%d PC %04x: warning - port 2 bit 1 set as output (OLVL) - not supported\n",cpu_getactivecpu(),activecpu_get_pc());
			}
			break;
		case 0x02:
			m6800.port1_data = data;
			if(m6800.port1_ddr == 0xff)
				M6800_io_write_byte_8(M6803_PORT1,m6800.port1_data);
			else
				M6800_io_write_byte_8(M6803_PORT1,(m6800.port1_data & m6800.port1_ddr)
					| (M6800_io_read_byte_8(M6803_PORT1) & (m6800.port1_ddr ^ 0xff)));
			break;
		case 0x03:
			m6800.port2_data = data;
			m6800.port2_ddr = data;
			if(m6800.port2_ddr == 0xff)
				M6800_io_write_byte_8(M6803_PORT2,m6800.port2_data);
			else
				M6800_io_write_byte_8(M6803_PORT2,(m6800.port2_data & m6800.port2_ddr)
					| (M6800_io_read_byte_8(M6803_PORT2) & (m6800.port2_ddr ^ 0xff)));
			break;
		case 0x04:
			if (m6800.port3_ddr != data)
			{
				m6800.port3_ddr = data;
				if(m6800.port3_ddr == 0xff)
					M6800_io_write_byte_8(M6803_PORT3,m6800.port3_data);
				else
					M6800_io_write_byte_8(M6803_PORT3,(m6800.port3_data & m6800.port3_ddr)
						| (M6800_io_read_byte_8(M6803_PORT3) & (m6800.port3_ddr ^ 0xff)));
			}
			break;
		case 0x05:
			if (m6800.port4_ddr != data)
			{
				m6800.port4_ddr = data;
				if(m6800.port4_ddr == 0xff)
					M6800_io_write_byte_8(M6803_PORT4,m6800.port4_data);
				else
					M6800_io_write_byte_8(M6803_PORT4,(m6800.port4_data & m6800.port4_ddr)
						| (M6800_io_read_byte_8(M6803_PORT4) & (m6800.port4_ddr ^ 0xff)));
			}
			break;
		case 0x06:
			m6800.port3_data = data;
			if(m6800.port3_ddr == 0xff)
				M6800_io_write_byte_8(M6803_PORT3,m6800.port3_data);
			else
				M6800_io_write_byte_8(M6803_PORT3,(m6800.port3_data & m6800.port3_ddr)
					| (M6800_io_read_byte_8(M6803_PORT3) & (m6800.port3_ddr ^ 0xff)));
			break;
		case 0x07:
			m6800.port4_data = data;
			if(m6800.port4_ddr == 0xff)
				M6800_io_write_byte_8(M6803_PORT4,m6800.port4_data);
			else
				M6800_io_write_byte_8(M6803_PORT4,(m6800.port4_data & m6800.port4_ddr)
					| (M6800_io_read_byte_8(M6803_PORT4) & (m6800.port4_ddr ^ 0xff)));
			break;
		case 0x08:
			m6800.tcsr = data;
			m6800.pending_tcsr &= m6800.tcsr;
			MODIFIED_tcsr;
			if( !(CC & 0x10) )
				CHECK_IRQ2;
//logerror("CPU #%d PC %04x: TCSR = %02x\n",cpu_getactivecpu(),activecpu_get_pc(),data);
			break;
		case 0x09:
			latch09 = data & 0xff;	/* 6301 only */
			CT  = 0xfff8;
			TOH = CTH;
			MODIFIED_counters;
			break;
		case 0x0a:	/* 6301 only */
			CT = (latch09 << 8) | (data & 0xff);
			TOH = CTH;
			MODIFIED_counters;
			break;
		case 0x0b:
			if( m6800.output_compare.b.h != data)
			{
				m6800.output_compare.b.h = data;
				MODIFIED_counters;
			}
			break;
		case 0x0c:
			if( m6800.output_compare.b.l != data)
			{
				m6800.output_compare.b.l = data;
				MODIFIED_counters;
			}
			break;
		case 0x0d:
		case 0x0e:
//			logerror("CPU #%d PC %04x: warning - write %02x to read only internal register %02x\n",cpu_getactivecpu(),activecpu_get_pc(),data,offset);
			break;
		case 0x0f:
		case 0x10:
		case 0x11:
		case 0x12:
		case 0x13:
//			logerror("CPU #%d PC %04x: warning - write %02x to unsupported internal register %02x\n",cpu_getactivecpu(),activecpu_get_pc(),data,offset);
			break;
		case 0x14:
//			logerror("CPU #%d PC %04x: write %02x to RAM control register\n",cpu_getactivecpu(),activecpu_get_pc(),data);
			m6800.ram_ctrl = data;
			break;
		case 0x15:
		case 0x16:
		case 0x17:
		case 0x18:
		case 0x19:
		case 0x1a:
		case 0x1b:
		case 0x1c:
		case 0x1d:
		case 0x1e:
		case 0x1f:
		default:
//			logerror("CPU #%d PC %04x: warning - write %02x to reserved internal register %02x\n",cpu_getactivecpu(),activecpu_get_pc(),data,offset);
			break;
	}
}
#endif

#if 0
/**************************************************************************
 * Generic set_info
 **************************************************************************/

static void m6800_set_info(UINT32 state, cpuinfo *info)
{
	switch (state)
	{
		/* --- the following bits of info are set as 64-bit signed integers --- */
		case CPUINFO_INT_INPUT_STATE + M6800_IRQ_LINE:	set_irq_line(M6800_IRQ_LINE, info->i);	break;
		case CPUINFO_INT_INPUT_STATE + M6800_TIN_LINE:	set_irq_line(M6800_TIN_LINE, info->i);	break;
		case CPUINFO_INT_INPUT_STATE + M6800_INPUT_LINE_NMI:	set_irq_line(M6800_INPUT_LINE_NMI, info->i);	break;

		case CPUINFO_INT_PC:							PC = info->i; CHANGE_PC();				break;
		case CPUINFO_INT_REGISTER + M6800_PC:			m6800.pc.w.l = info->i;					break;
		case CPUINFO_INT_SP:							S = info->i;							break;
		case CPUINFO_INT_REGISTER + M6800_S:			m6800.s.w.l = info->i;					break;
		case CPUINFO_INT_REGISTER + M6800_CC:			m6800.cc = info->i;						break;
		case CPUINFO_INT_REGISTER + M6800_A:			m6800.d.b.h = info->i;					break;
		case CPUINFO_INT_REGISTER + M6800_B:			m6800.d.b.l = info->i;					break;
		case CPUINFO_INT_REGISTER + M6800_X:			m6800.x.w.l = info->i;					break;
	}
}



/**************************************************************************
 * Generic get_info
 **************************************************************************/

void m6800_get_info(UINT32 state, cpuinfo *info)
{
	switch (state)
	{
		/* --- the following bits of info are returned as 64-bit signed integers --- */
		case CPUINFO_INT_CONTEXT_SIZE:					info->i = sizeof(m6800);				break;
		case CPUINFO_INT_INPUT_LINES:					info->i = 2;							break;
		case CPUINFO_INT_DEFAULT_IRQ_VECTOR:			info->i = 0;							break;
		case CPUINFO_INT_ENDIANNESS:					info->i = CPU_IS_BE;					break;
		case CPUINFO_INT_CLOCK_DIVIDER:					info->i = 1;							break;
		case CPUINFO_INT_MIN_INSTRUCTION_BYTES:			info->i = 1;							break;
		case CPUINFO_INT_MAX_INSTRUCTION_BYTES:			info->i = 4;							break;
		case CPUINFO_INT_MIN_CYCLES:					info->i = 1;							break;
		case CPUINFO_INT_MAX_CYCLES:					info->i = 12;							break;

		case CPUINFO_INT_DATABUS_WIDTH + ADDRESS_SPACE_PROGRAM:	info->i = 8;					break;
		case CPUINFO_INT_ADDRBUS_WIDTH + ADDRESS_SPACE_PROGRAM: info->i = 16;					break;
		case CPUINFO_INT_ADDRBUS_SHIFT + ADDRESS_SPACE_PROGRAM: info->i = 0;					break;
		case CPUINFO_INT_DATABUS_WIDTH + ADDRESS_SPACE_DATA:	info->i = 0;					break;
		case CPUINFO_INT_ADDRBUS_WIDTH + ADDRESS_SPACE_DATA: 	info->i = 0;					break;
		case CPUINFO_INT_ADDRBUS_SHIFT + ADDRESS_SPACE_DATA: 	info->i = 0;					break;
		case CPUINFO_INT_DATABUS_WIDTH + ADDRESS_SPACE_IO:		info->i = 0;					break;
		case CPUINFO_INT_ADDRBUS_WIDTH + ADDRESS_SPACE_IO: 		info->i = 0;					break;
		case CPUINFO_INT_ADDRBUS_SHIFT + ADDRESS_SPACE_IO: 		info->i = 0;					break;

		case CPUINFO_INT_INPUT_STATE + M6800_IRQ_LINE:	info->i = m6800.irq_state[M6800_IRQ_LINE]; break;
		case CPUINFO_INT_INPUT_STATE + M6800_TIN_LINE:	info->i = m6800.irq_state[M6800_TIN_LINE]; break;
		case CPUINFO_INT_INPUT_STATE + M6800_INPUT_LINE_NMI:	info->i = m6800.nmi_state;				break;

		case CPUINFO_INT_PREVIOUSPC:					info->i = m6800.ppc.w.l;				break;

		case CPUINFO_INT_PC:							info->i = PC;							break;
		case CPUINFO_INT_REGISTER + M6800_PC:			info->i = m6800.pc.w.l;					break;
		case CPUINFO_INT_SP:							info->i = S;							break;
		case CPUINFO_INT_REGISTER + M6800_S:			info->i = m6800.s.w.l;					break;
		case CPUINFO_INT_REGISTER + M6800_CC:			info->i = m6800.cc;						break;
		case CPUINFO_INT_REGISTER + M6800_A:			info->i = m6800.d.b.h;					break;
		case CPUINFO_INT_REGISTER + M6800_B:			info->i = m6800.d.b.l;					break;
		case CPUINFO_INT_REGISTER + M6800_X:			info->i = m6800.x.w.l;					break;
		case CPUINFO_INT_REGISTER + M6800_WAI_STATE:	info->i = m6800.wai_state;				break;

		/* --- the following bits of info are returned as pointers to data or functions --- */
		case CPUINFO_PTR_SET_INFO:						info->setinfo = m6800_set_info;			break;
		case CPUINFO_PTR_GET_CONTEXT:					info->getcontext = m6800_get_context;	break;
		case CPUINFO_PTR_SET_CONTEXT:					info->setcontext = m6800_set_context;	break;
		case CPUINFO_PTR_INIT:							info->init = m6800_init;				break;
		case CPUINFO_PTR_RESET:							info->reset = m6800_reset;				break;
		case CPUINFO_PTR_EXIT:							info->exit = m6800_exit;				break;
		case CPUINFO_PTR_EXECUTE:						info->execute = m6800_execute;			break;
		case CPUINFO_PTR_BURN:							info->burn = NULL;						break;
#ifdef MAME_DEBUG
		case CPUINFO_PTR_DISASSEMBLE:					info->disassemble = m6800_dasm;			break;
#endif /* MAME_DEBUG */
		case CPUINFO_PTR_INSTRUCTION_COUNTER:			info->icount = &m6800_ICount;			break;

		/* --- the following bits of info are returned as NULL-terminated strings --- */
		case CPUINFO_STR_NAME:							strcpy(info->s, "M6800");				break;
		case CPUINFO_STR_CORE_FAMILY:					strcpy(info->s, "Motorola 6800");		break;
		case CPUINFO_STR_CORE_VERSION:					strcpy(info->s, "1.1");					break;
		case CPUINFO_STR_CORE_FILE:						strcpy(info->s, __FILE__);				break;
		case CPUINFO_STR_CORE_CREDITS:					strcpy(info->s, "The MAME team.");		break;

		case CPUINFO_STR_FLAGS:
			sprintf(info->s, "%c%c%c%c%c%c%c%c",
				m6800.cc & 0x80 ? '?':'.',
				m6800.cc & 0x40 ? '?':'.',
				m6800.cc & 0x20 ? 'H':'.',
				m6800.cc & 0x10 ? 'I':'.',
				m6800.cc & 0x08 ? 'N':'.',
				m6800.cc & 0x04 ? 'Z':'.',
				m6800.cc & 0x02 ? 'V':'.',
				m6800.cc & 0x01 ? 'C':'.');
			break;

		case CPUINFO_STR_REGISTER + M6800_A:			sprintf(info->s, "A:%02X", m6800.d.b.h); break;
		case CPUINFO_STR_REGISTER + M6800_B:			sprintf(info->s, "B:%02X", m6800.d.b.l); break;
		case CPUINFO_STR_REGISTER + M6800_PC:			sprintf(info->s, "PC:%04X", m6800.pc.w.l); break;
		case CPUINFO_STR_REGISTER + M6800_S:			sprintf(info->s, "S:%04X", m6800.s.w.l); break;
		case CPUINFO_STR_REGISTER + M6800_X:			sprintf(info->s, "X:%04X", m6800.x.w.l); break;
		case CPUINFO_STR_REGISTER + M6800_CC:			sprintf(info->s, "CC:%02X", m6800.cc); break;
		case CPUINFO_STR_REGISTER + M6800_WAI_STATE:	sprintf(info->s, "WAI:%X", m6800.wai_state); break;
	}
}


#if (HAS_M6801)
/**************************************************************************
 * CPU-specific set_info
 **************************************************************************/

void m6801_get_info(UINT32 state, cpuinfo *info)
{
	switch (state)
	{
		/* --- the following bits of info are returned as 64-bit signed integers --- */
		case CPUINFO_INT_DATABUS_WIDTH + ADDRESS_SPACE_IO:		info->i = 8;					break;
		case CPUINFO_INT_ADDRBUS_WIDTH + ADDRESS_SPACE_IO: 		info->i = 9;					break;

		/* --- the following bits of info are returned as pointers to data or functions --- */
		case CPUINFO_PTR_INIT:							info->init = m6801_init;				break;
		case CPUINFO_PTR_EXECUTE:						info->execute = m6803_execute;			break;
#ifdef MAME_DEBUG
		case CPUINFO_PTR_DISASSEMBLE:					info->disassemble = m6801_dasm;			break;
#endif /* MAME_DEBUG */

		/* --- the following bits of info are returned as NULL-terminated strings --- */
		case CPUINFO_STR_NAME:							strcpy(info->s, "M6801");				break;

		default:										m6800_get_info(state, info);			break;
	}
}
#endif


#if (HAS_M6802)
/**************************************************************************
 * CPU-specific set_info
 **************************************************************************/

void m6802_get_info(UINT32 state, cpuinfo *info)
{
	switch (state)
	{
		/* --- the following bits of info are returned as pointers to data or functions --- */
		case CPUINFO_PTR_INIT:							info->init = m6802_init;				break;
#ifdef MAME_DEBUG
		case CPUINFO_PTR_DISASSEMBLE:					info->disassemble = m6802_dasm;			break;
#endif /* MAME_DEBUG */

		/* --- the following bits of info are returned as NULL-terminated strings --- */
		case CPUINFO_STR_NAME:							strcpy(info->s, "M6802");				break;

		default:										m6800_get_info(state, info);			break;
	}
}
#endif


#if (HAS_M6803)
/**************************************************************************
 * CPU-specific set_info
 **************************************************************************/

void m6803_get_info(UINT32 state, cpuinfo *info)
{
	switch (state)
	{
		/* --- the following bits of info are returned as 64-bit signed integers --- */
		case CPUINFO_INT_DATABUS_WIDTH + ADDRESS_SPACE_IO:		info->i = 8;					break;
		case CPUINFO_INT_ADDRBUS_WIDTH + ADDRESS_SPACE_IO: 		info->i = 9;					break;

		/* --- the following bits of info are returned as pointers to data or functions --- */
		case CPUINFO_PTR_INIT:							info->init = m6803_init;				break;
		case CPUINFO_PTR_EXECUTE:						info->execute = m6803_execute;			break;
#ifdef MAME_DEBUG
		case CPUINFO_PTR_DISASSEMBLE:					info->disassemble = m6803_dasm;			break;
#endif /* MAME_DEBUG */

		case CPUINFO_PTR_INTERNAL_MEMORY_MAP + ADDRESS_SPACE_PROGRAM: info->internal_map = construct_map_m6803_mem; break;

		/* --- the following bits of info are returned as NULL-terminated strings --- */
		case CPUINFO_STR_NAME:							strcpy(info->s, "M6803");				break;

		default:										m6800_get_info(state, info);			break;
	}
}
#endif


#if (HAS_M6808)
/**************************************************************************
 * CPU-specific set_info
 **************************************************************************/

void m6808_get_info(UINT32 state, cpuinfo *info)
{
	switch (state)
	{
		/* --- the following bits of info are returned as pointers to data or functions --- */
		case CPUINFO_PTR_INIT:							info->init = m6808_init;				break;
#ifdef MAME_DEBUG
		case CPUINFO_PTR_DISASSEMBLE:					info->disassemble = m6808_dasm;			break;
#endif /* MAME_DEBUG */

		/* --- the following bits of info are returned as NULL-terminated strings --- */
		case CPUINFO_STR_NAME:							strcpy(info->s, "M6808");				break;

		default:										m6800_get_info(state, info);			break;
	}
}
#endif


#if (HAS_HD63701)
/**************************************************************************
 * CPU-specific set_info
 **************************************************************************/

void hd63701_get_info(UINT32 state, cpuinfo *info)
{
	switch (state)
	{
		/* --- the following bits of info are returned as 64-bit signed integers --- */
		case CPUINFO_INT_DATABUS_WIDTH + ADDRESS_SPACE_IO:		info->i = 8;					break;
		case CPUINFO_INT_ADDRBUS_WIDTH + ADDRESS_SPACE_IO: 		info->i = 9;					break;

		/* --- the following bits of info are returned as pointers to data or functions --- */
		case CPUINFO_PTR_INIT:							info->init = hd63701_init;				break;
		case CPUINFO_PTR_EXECUTE:						info->execute = hd63701_execute;		break;
#ifdef MAME_DEBUG
		case CPUINFO_PTR_DISASSEMBLE:					info->disassemble = hd63701_dasm;		break;
#endif /* MAME_DEBUG */

		/* --- the following bits of info are returned as NULL-terminated strings --- */
		case CPUINFO_STR_NAME:							strcpy(info->s, "HD63701");				break;

		default:										m6800_get_info(state, info);			break;
	}
}
#endif


#if (HAS_NSC8105)
/**************************************************************************
 * CPU-specific set_info
 **************************************************************************/

void nsc8105_get_info(UINT32 state, cpuinfo *info)
{
	switch (state)
	{
		/* --- the following bits of info are returned as pointers to data or functions --- */
		case CPUINFO_PTR_INIT:							info->init = nsc8105_init;				break;
		case CPUINFO_PTR_EXECUTE:						info->execute = nsc8105_execute;		break;
#ifdef MAME_DEBUG
		case CPUINFO_PTR_DISASSEMBLE:					info->disassemble = nsc8105_dasm;		break;
#endif /* MAME_DEBUG */

		/* --- the following bits of info are returned as NULL-terminated strings --- */
		case CPUINFO_STR_NAME:							strcpy(info->s, "NSC8105");				break;

		default:										m6800_get_info(state, info);			break;
	}
}
#endif

#endif
