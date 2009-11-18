/*** m6809: Portable 6809 emulator ******************************************

    Copyright John Butler

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

    History:
991026 HJB:
    Fixed missing calls to cpu_changepc() for the TFR and EXG ocpodes.
    Replaced m6809_slapstic checks by a macro (CHANGE_PC). ESB still
    needs the tweaks.

991024 HJB:
    Tried to improve speed: Using bit7 of cycles1/2 as flag for multi
    byte opcodes is gone, those opcodes now call fetch_effective_address().
    Got rid of the slow/fast flags for stack (S and U) memory accesses.
    Minor changes to use 32 bit values as arguments to memory functions
    and added defines for that purpose (e.g. X = 16bit XD = 32bit).

990312 HJB:
    Added bugfixes according to Aaron's findings.
    Reset only sets CC_II and CC_IF, DP to zero and PC from reset vector.
990311 HJB:
    Added _info functions. Now uses static m6808_Regs struct instead
    of single statics. Changed the 16 bit registers to use the generic
    PAIR union. Registers defined using macros. Split the core into
    four execution loops for M6802, M6803, M6808 and HD63701.
    TST, TSTA and TSTB opcodes reset carry flag.
    Modified the read/write stack handlers to push LSB first then MSB
    and pull MSB first then LSB.

990228 HJB:
    Changed the interrupt handling again. Now interrupts are taken
    either right at the moment the lines are asserted or whenever
    an interrupt is enabled and the corresponding line is still
    asserted. That way the pending_interrupts checks are not
    needed anymore. However, the CWAI and SYNC flags still need
    some flags, so I changed the name to 'int_state'.
    This core also has the code for the old interrupt system removed.

990225 HJB:
    Cleaned up the code here and there, added some comments.
    Slightly changed the SAR opcodes (similiar to other CPU cores).
    Added symbolic names for the flag bits.
    Changed the way CWAI/Interrupt() handle CPU state saving.
    A new flag M6809_STATE in pending_interrupts is used to determine
    if a state save is needed on interrupt entry or already done by CWAI.
    Added M6809_IRQ_LINE and M6809_FIRQ_LINE defines to m6809.h
    Moved the internal interrupt_pending flags from m6809.h to m6809.c
    Changed CWAI cycles2[0x3c] to be 2 (plus all or at least 19 if
    CWAI actually pushes the entire state).
    Implemented undocumented TFR/EXG for undefined source and mixed 8/16
    bit transfers (they should transfer/exchange the constant $ff).
    Removed unused jmp/jsr _slap functions from 6809ops.c,
    m6809_slapstick check moved into the opcode functions.

*****************************************************************************/

//#include "debugger.h"
#include "burnint.h"
#include "m6809.h"

/* Enable big switch statement for the main opcodes */
#ifndef BIG_SWITCH
#define BIG_SWITCH  1
#endif

#define VERBOSE 0

#define LOG(x)	do { if (VERBOSE) logerror x; } while (0)

//extern offs_t m6809_dasm(char *buffer, offs_t pc, const UINT8 *oprom, const UINT8 *opram);

#define M6809_INLINE		static
#define change_pc(newpc)	m6809.pc.w.l = (newpc)
#define M6809_CLEAR_LINE	0
#define M6809_INPUT_LINE_NMI	32

M6809_INLINE void fetch_effective_address( void );

/* flag bits in the cc register */
#define CC_C    0x01        /* Carry */
#define CC_V    0x02        /* Overflow */
#define CC_Z    0x04        /* Zero */
#define CC_N    0x08        /* Negative */
#define CC_II   0x10        /* Inhibit IRQ */
#define CC_H    0x20        /* Half (auxiliary) carry */
#define CC_IF   0x40        /* Inhibit FIRQ */
#define CC_E    0x80        /* entire state pushed */

/* 6809 registers */
static m6809_Regs m6809;

#define pPPC    m6809.ppc
#define pPC 	m6809.pc
#define pU		m6809.u
#define pS		m6809.s
#define pX		m6809.x
#define pY		m6809.y
#define pD		m6809.d

#define	wPPC		m6809.ppc.w.l
#define PC  	m6809.pc.w.l
#define PCD 	m6809.pc.d
#define U		m6809.u.w.l
#define UD		m6809.u.d
#define S		m6809.s.w.l
#define SD		m6809.s.d
#define X		m6809.x.w.l
#define XD		m6809.x.d
#define Y		m6809.y.w.l
#define YD		m6809.y.d
#define D   	m6809.d.w.l
#define A   	m6809.d.b.h
#define B		m6809.d.b.l
#define DP		m6809.dp.b.h
#define DPD 	m6809.dp.d
#define CC  	m6809.cc

static PAIR ea;         /* effective address */
#define EA	ea.w.l
#define EAD ea.d

#define CHANGE_PC change_pc(PCD)

#define M6809_CWAI		8	/* set when CWAI is waiting for an interrupt */
#define M6809_SYNC		16	/* set when SYNC is waiting for an interrupt */
#define M6809_LDS		32	/* set when LDS occured at least once */

#define CHECK_IRQ_LINES 												\
	if( m6809.irq_state[M6809_IRQ_LINE] != M6809_CLEAR_LINE ||				\
		m6809.irq_state[M6809_FIRQ_LINE] != M6809_CLEAR_LINE )				\
		m6809.int_state &= ~M6809_SYNC; /* clear SYNC flag */			\
	if( m6809.irq_state[M6809_FIRQ_LINE]!=M6809_CLEAR_LINE && !(CC & CC_IF) ) \
	{																	\
		/* fast IRQ */													\
		/* HJB 990225: state already saved by CWAI? */					\
		if( m6809.int_state & M6809_CWAI )								\
		{																\
			m6809.int_state &= ~M6809_CWAI;  /* clear CWAI */			\
			m6809.extra_cycles += 7;		 /* subtract +7 cycles */	\
        }                                                               \
		else															\
		{																\
			CC &= ~CC_E;				/* save 'short' state */        \
			PUSHWORD(pPC);												\
			PUSHBYTE(CC);												\
			m6809.extra_cycles += 10;	/* subtract +10 cycles */		\
		}																\
		CC |= CC_IF | CC_II;			/* inhibit FIRQ and IRQ */		\
		PCD=RM16(0xfff6);												\
		CHANGE_PC;														\
	}																	\
	else																\
	if( m6809.irq_state[M6809_IRQ_LINE]!=M6809_CLEAR_LINE && !(CC & CC_II) )	\
	{																	\
		/* standard IRQ */												\
		/* HJB 990225: state already saved by CWAI? */					\
		if( m6809.int_state & M6809_CWAI )								\
		{																\
			m6809.int_state &= ~M6809_CWAI;  /* clear CWAI flag */		\
			m6809.extra_cycles += 7;		 /* subtract +7 cycles */	\
		}																\
		else															\
		{																\
			CC |= CC_E; 				/* save entire state */ 		\
			PUSHWORD(pPC);												\
			PUSHWORD(pU);												\
			PUSHWORD(pY);												\
			PUSHWORD(pX);												\
			PUSHBYTE(DP);												\
			PUSHBYTE(B);												\
			PUSHBYTE(A);												\
			PUSHBYTE(CC);												\
			m6809.extra_cycles += 19;	 /* subtract +19 cycles */		\
		}																\
		CC |= CC_II;					/* inhibit IRQ */				\
		PCD=RM16(0xfff8);												\
		CHANGE_PC;														\
	}

/* public globals */
static int m6809_ICount;

/* these are re-defined in m6809.h TO RAM, ROM or functions in cpuintrf.c */
#define RM(Addr)		M6809_RDMEM(Addr)
#define WM(Addr,Value)	M6809_WRMEM(Addr,Value)
#define ROP(Addr)		M6809_RDOP(Addr)
#define ROP_ARG(Addr)	M6809_RDOP_ARG(Addr)

/* macros to access memory */
#define IMMBYTE(b)	b = ROP_ARG(PCD); PC++
#define IMMWORD(w)	w.d = (ROP_ARG(PCD)<<8) | ROP_ARG((PCD+1)&0xffff); PC+=2

#define PUSHBYTE(b) --S; WM(SD,b)
#define PUSHWORD(w) --S; WM(SD,w.b.l); --S; WM(SD,w.b.h)
#define PULLBYTE(b) b = RM(SD); S++
#define PULLWORD(w) w = RM(SD)<<8; S++; w |= RM(SD); S++

#define PSHUBYTE(b) --U; WM(UD,b);
#define PSHUWORD(w) --U; WM(UD,w.b.l); --U; WM(UD,w.b.h)
#define PULUBYTE(b) b = RM(UD); U++
#define PULUWORD(w) w = RM(UD)<<8; U++; w |= RM(UD); U++

#define CLR_HNZVC   CC&=~(CC_H|CC_N|CC_Z|CC_V|CC_C)
#define CLR_NZV 	CC&=~(CC_N|CC_Z|CC_V)
#define CLR_NZ		CC&=~(CC_N|CC_Z)
#define CLR_HNZC	CC&=~(CC_H|CC_N|CC_Z|CC_C)
#define CLR_NZVC	CC&=~(CC_N|CC_Z|CC_V|CC_C)
#define CLR_Z		CC&=~(CC_Z)
#define CLR_NZC 	CC&=~(CC_N|CC_Z|CC_C)
#define CLR_ZC		CC&=~(CC_Z|CC_C)

/* macros for CC -- CC bits affected should be reset before calling */
#define SET_Z(a)		if(!a)SEZ
#define SET_Z8(a)		SET_Z((UINT8)a)
#define SET_Z16(a)		SET_Z((UINT16)a)
#define SET_N8(a)		CC|=((a&0x80)>>4)
#define SET_N16(a)		CC|=((a&0x8000)>>12)
#define SET_H(a,b,r)	CC|=(((a^b^r)&0x10)<<1)
#define SET_C8(a)		CC|=((a&0x100)>>8)
#define SET_C16(a)		CC|=((a&0x10000)>>16)
#define SET_V8(a,b,r)	CC|=(((a^b^r^(r>>1))&0x80)>>6)
#define SET_V16(a,b,r)	CC|=(((a^b^r^(r>>1))&0x8000)>>14)

static const UINT8 flags8i[256]=	 /* increment */
{
CC_Z,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
CC_N|CC_V,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,
CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,
CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,
CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,
CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,
CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,
CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,
CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N
};
static const UINT8 flags8d[256]= /* decrement */
{
CC_Z,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,CC_V,
CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,
CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,
CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,
CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,
CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,
CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,
CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,
CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N,CC_N
};
#define SET_FLAGS8I(a)		{CC|=flags8i[(a)&0xff];}
#define SET_FLAGS8D(a)		{CC|=flags8d[(a)&0xff];}

/* combos */
#define SET_NZ8(a)			{SET_N8(a);SET_Z(a);}
#define SET_NZ16(a)			{SET_N16(a);SET_Z(a);}
#define SET_FLAGS8(a,b,r)	{SET_N8(r);SET_Z8(r);SET_V8(a,b,r);SET_C8(r);}
#define SET_FLAGS16(a,b,r)	{SET_N16(r);SET_Z16(r);SET_V16(a,b,r);SET_C16(r);}

/* for treating an unsigned byte as a signed word */
#define SIGNED(b) ((UINT16)(b&0x80?b|0xff00:b))

/* macros for addressing modes (postbytes have their own code) */
#define DIRECT	EAD = DPD; IMMBYTE(ea.b.l)
#define IMM8	EAD = PCD; PC++
#define IMM16	EAD = PCD; PC+=2
#define EXTENDED IMMWORD(ea)

/* macros to set status flags */
#if defined(SEC)
#undef SEC
#endif
#define SEC CC|=CC_C
#define CLC CC&=~CC_C
#define SEZ CC|=CC_Z
#define CLZ CC&=~CC_Z
#define SEN CC|=CC_N
#define CLN CC&=~CC_N
#define SEV CC|=CC_V
#define CLV CC&=~CC_V
#define SEH CC|=CC_H
#define CLH CC&=~CC_H

/* macros for convenience */
#define DIRBYTE(b) {DIRECT;b=RM(EAD);}
#define DIRWORD(w) {DIRECT;w.d=RM16(EAD);}
#define EXTBYTE(b) {EXTENDED;b=RM(EAD);}
#define EXTWORD(w) {EXTENDED;w.d=RM16(EAD);}

/* macros for branch instructions */
#define BRANCH(f) { 					\
	UINT8 t;							\
	IMMBYTE(t); 						\
	if( f ) 							\
	{									\
		PC += SIGNED(t);				\
		CHANGE_PC;						\
	}									\
}

#define LBRANCH(f) {                    \
	PAIR t; 							\
	IMMWORD(t); 						\
	if( f ) 							\
	{									\
		m6809_ICount -= 1;				\
		PC += t.w.l;					\
		CHANGE_PC;						\
	}									\
}

#define NXORV  ((CC&CC_N)^((CC&CC_V)<<2))

/* macros for setting/getting registers in TFR/EXG instructions */

#if (!BIG_SWITCH)
/* timings for 1-byte opcodes */
static const UINT8 cycles1[] =
{
	/*   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F */
  /*0*/  6, 0, 0, 6, 6, 0, 6, 6, 6, 6, 6, 0, 6, 6, 3, 6,
  /*1*/  0, 0, 2, 4, 0, 0, 5, 9, 0, 2, 3, 0, 3, 2, 8, 6,
  /*2*/  3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
  /*3*/  4, 4, 4, 4, 5, 5, 5, 5, 0, 5, 3, 6,20,11, 0,19,
  /*4*/  2, 0, 0, 2, 2, 0, 2, 2, 2, 2, 2, 0, 2, 2, 0, 2,
  /*5*/  2, 0, 0, 2, 2, 0, 2, 2, 2, 2, 2, 0, 2, 2, 0, 2,
  /*6*/  6, 0, 0, 6, 6, 0, 6, 6, 6, 6, 6, 0, 6, 6, 3, 6,
  /*7*/  7, 0, 0, 7, 7, 0, 7, 7, 7, 7, 7, 0, 7, 7, 4, 7,
  /*8*/  2, 2, 2, 4, 2, 2, 2, 2, 2, 2, 2, 2, 4, 7, 3, 0,
  /*9*/  4, 4, 4, 6, 4, 4, 4, 4, 4, 4, 4, 4, 6, 7, 5, 5,
  /*A*/  4, 4, 4, 6, 4, 4, 4, 4, 4, 4, 4, 4, 6, 7, 5, 5,
  /*B*/  5, 5, 5, 7, 5, 5, 5, 5, 5, 5, 5, 5, 7, 8, 6, 6,
  /*C*/  2, 2, 2, 4, 2, 2, 2, 2, 2, 2, 2, 2, 3, 0, 3, 3,
  /*D*/  4, 4, 4, 6, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5,
  /*E*/  4, 4, 4, 6, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5,
  /*F*/  5, 5, 5, 7, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6
};
#endif

M6809_INLINE UINT32 RM16( UINT32 Addr )
{
	UINT32 result = RM(Addr) << 8;
	return result | RM((Addr+1)&0xffff);
}

M6809_INLINE void WM16( UINT32 Addr, PAIR *p )
{
	WM( Addr, p->b.h );
	WM( (Addr+1)&0xffff, p->b.l );
}

/****************************************************************************
 * Get all registers in given buffer
 ****************************************************************************/
void m6809_get_context(void *dst)
{
	if( dst )
		*(m6809_Regs*)dst = m6809;
}

/****************************************************************************
 * Set all registers to given values
 ****************************************************************************/
void m6809_set_context(void *src)
{
	if( src )
		m6809 = *(m6809_Regs*)src;
	CHANGE_PC;

    CHECK_IRQ_LINES;
}


/****************************************************************************/
/* Reset registers to their initial values                                  */
/****************************************************************************/
void m6809_init(int (*irqcallback)(int))
{
//	state_save_register_item("m6809", index, PC);
//	state_save_register_item("m6809", index, PPC);
//	state_save_register_item("m6809", index, D);
//	state_save_register_item("m6809", index, DP);
//	state_save_register_item("m6809", index, U);
//	state_save_register_item("m6809", index, S);
//	state_save_register_item("m6809", index, X);
//	state_save_register_item("m6809", index, Y);
//	state_save_register_item("m6809", index, CC);
//	state_save_register_item_array("m6809", index, m6809.irq_state);
//	state_save_register_item("m6809", index, m6809.int_state);
//	state_save_register_item("m6809", index, m6809.nmi_state);

	m6809.irq_callback = irqcallback;
}

void m6809_reset(void)
{
	m6809.int_state = 0;
	m6809.nmi_state = M6809_CLEAR_LINE;
	m6809.irq_state[0] = M6809_CLEAR_LINE;
	m6809.irq_state[1] = M6809_CLEAR_LINE;

	DPD = 0;			/* Reset direct page register */

    CC |= CC_II;        /* IRQ disabled */
    CC |= CC_IF;        /* FIRQ disabled */

	PCD = RM16(0xfffe);
	CHANGE_PC;
}

/*
static void m6809_exit(void)
{

}*/

/* Generate interrupts */
/****************************************************************************
 * Set IRQ line state
 ****************************************************************************/
void m6809_set_irq_line(int irqline, int state)
{
	if (irqline == M6809_INPUT_LINE_NMI)
	{
		if (m6809.nmi_state == state) return;
		m6809.nmi_state = state;
//		LOG(("M6809#%d set_irq_line (NMI) %d\n", cpu_getactivecpu(), state));
		if( state == M6809_CLEAR_LINE ) return;

		/* if the stack was not yet initialized */
	    if( !(m6809.int_state & M6809_LDS) ) return;

	    m6809.int_state &= ~M6809_SYNC;
		/* HJB 990225: state already saved by CWAI? */
		if( m6809.int_state & M6809_CWAI )
		{
			m6809.int_state &= ~M6809_CWAI;
			m6809.extra_cycles += 7;	/* subtract +7 cycles next time */
	    }
		else
		{
			CC |= CC_E; 				/* save entire state */
			PUSHWORD(pPC);
			PUSHWORD(pU);
			PUSHWORD(pY);
			PUSHWORD(pX);
			PUSHBYTE(DP);
			PUSHBYTE(B);
			PUSHBYTE(A);
			PUSHBYTE(CC);
			m6809.extra_cycles += 19;	/* subtract +19 cycles next time */
		}
		CC |= CC_IF | CC_II;			/* inhibit FIRQ and IRQ */
		PCD = RM16(0xfffc);
		CHANGE_PC;
	}
	else if (irqline < 2)
	{
//	    LOG(("M6809#%d set_irq_line %d, %d\n", cpu_getactivecpu(), irqline, state));
		m6809.irq_state[irqline] = state;
		if (state == M6809_CLEAR_LINE) return;
		CHECK_IRQ_LINES;
	}
}

#ifdef __cplusplus
extern "C" {
#endif

M6809_INLINE void abx(void);
M6809_INLINE void adca_di(void);
M6809_INLINE void adca_ex(void);
M6809_INLINE void adca_im(void);
M6809_INLINE void adca_ix(void);
M6809_INLINE void adcb_di(void);
M6809_INLINE void adcb_ex(void);
M6809_INLINE void adcb_im(void);
M6809_INLINE void adcb_ix(void);
M6809_INLINE void adda_di(void);
M6809_INLINE void adda_ex(void);
M6809_INLINE void adda_im(void);
M6809_INLINE void adda_ix(void);
M6809_INLINE void addb_di(void);
M6809_INLINE void addb_ex(void);
M6809_INLINE void addb_im(void);
M6809_INLINE void addb_ix(void);
M6809_INLINE void addd_di(void);
M6809_INLINE void addd_ex(void);
M6809_INLINE void addd_im(void);
M6809_INLINE void addd_ix(void);
M6809_INLINE void anda_di(void);
M6809_INLINE void anda_ex(void);
M6809_INLINE void anda_im(void);
M6809_INLINE void anda_ix(void);
M6809_INLINE void andb_di(void);
M6809_INLINE void andb_ex(void);
M6809_INLINE void andb_im(void);
M6809_INLINE void andb_ix(void);
M6809_INLINE void andcc(void);
M6809_INLINE void asl_di(void);
M6809_INLINE void asl_ex(void);
M6809_INLINE void asl_ix(void);
M6809_INLINE void asla(void);
M6809_INLINE void aslb(void);
M6809_INLINE void asr_di(void);
M6809_INLINE void asr_ex(void);
M6809_INLINE void asr_ix(void);
M6809_INLINE void asra(void);
M6809_INLINE void asrb(void);
M6809_INLINE void bcc(void);
M6809_INLINE void bcs(void);
M6809_INLINE void beq(void);
M6809_INLINE void bge(void);
M6809_INLINE void bgt(void);
M6809_INLINE void bhi(void);
M6809_INLINE void bita_di(void);
M6809_INLINE void bita_ex(void);
M6809_INLINE void bita_im(void);
M6809_INLINE void bita_ix(void);
M6809_INLINE void bitb_di(void);
M6809_INLINE void bitb_ex(void);
M6809_INLINE void bitb_im(void);
M6809_INLINE void bitb_ix(void);
M6809_INLINE void ble(void);
M6809_INLINE void bls(void);
M6809_INLINE void blt(void);
M6809_INLINE void bmi(void);
M6809_INLINE void bne(void);
M6809_INLINE void bpl(void);
M6809_INLINE void bra(void);
M6809_INLINE void brn(void);
M6809_INLINE void bsr(void);
M6809_INLINE void bvc(void);
M6809_INLINE void bvs(void);
M6809_INLINE void clr_di(void);
M6809_INLINE void clr_ex(void);
M6809_INLINE void clr_ix(void);
M6809_INLINE void clra(void);
M6809_INLINE void clrb(void);
M6809_INLINE void cmpa_di(void);
M6809_INLINE void cmpa_ex(void);
M6809_INLINE void cmpa_im(void);
M6809_INLINE void cmpa_ix(void);
M6809_INLINE void cmpb_di(void);
M6809_INLINE void cmpb_ex(void);
M6809_INLINE void cmpb_im(void);
M6809_INLINE void cmpb_ix(void);
M6809_INLINE void cmpd_di(void);
M6809_INLINE void cmpd_ex(void);
M6809_INLINE void cmpd_im(void);
M6809_INLINE void cmpd_ix(void);
M6809_INLINE void cmps_di(void);
M6809_INLINE void cmps_ex(void);
M6809_INLINE void cmps_im(void);
M6809_INLINE void cmps_ix(void);
M6809_INLINE void cmpu_di(void);
M6809_INLINE void cmpu_ex(void);
M6809_INLINE void cmpu_im(void);
M6809_INLINE void cmpu_ix(void);
M6809_INLINE void cmpx_di(void);
M6809_INLINE void cmpx_ex(void);
M6809_INLINE void cmpx_im(void);
M6809_INLINE void cmpx_ix(void);
M6809_INLINE void cmpy_di(void);
M6809_INLINE void cmpy_ex(void);
M6809_INLINE void cmpy_im(void);
M6809_INLINE void cmpy_ix(void);
M6809_INLINE void com_di(void);
M6809_INLINE void com_ex(void);
M6809_INLINE void com_ix(void);
M6809_INLINE void coma(void);
M6809_INLINE void comb(void);
M6809_INLINE void cwai(void);
M6809_INLINE void daa(void);
M6809_INLINE void dec_di(void);
M6809_INLINE void dec_ex(void);
M6809_INLINE void dec_ix(void);
M6809_INLINE void deca(void);
M6809_INLINE void decb(void);
M6809_INLINE void eora_di(void);
M6809_INLINE void eora_ex(void);
M6809_INLINE void eora_im(void);
M6809_INLINE void eora_ix(void);
M6809_INLINE void eorb_di(void);
M6809_INLINE void eorb_ex(void);
M6809_INLINE void eorb_im(void);
M6809_INLINE void eorb_ix(void);
M6809_INLINE void exg(void);
M6809_INLINE void illegal(void);
M6809_INLINE void inc_di(void);
M6809_INLINE void inc_ex(void);
M6809_INLINE void inc_ix(void);
M6809_INLINE void inca(void);
M6809_INLINE void incb(void);
M6809_INLINE void jmp_di(void);
M6809_INLINE void jmp_ex(void);
M6809_INLINE void jmp_ix(void);
M6809_INLINE void jsr_di(void);
M6809_INLINE void jsr_ex(void);
M6809_INLINE void jsr_ix(void);
M6809_INLINE void lbcc(void);
M6809_INLINE void lbcs(void);
M6809_INLINE void lbeq(void);
M6809_INLINE void lbge(void);
M6809_INLINE void lbgt(void);
M6809_INLINE void lbhi(void);
M6809_INLINE void lble(void);
M6809_INLINE void lbls(void);
M6809_INLINE void lblt(void);
M6809_INLINE void lbmi(void);
M6809_INLINE void lbne(void);
M6809_INLINE void lbpl(void);
M6809_INLINE void lbra(void);
M6809_INLINE void lbrn(void);
M6809_INLINE void lbsr(void);
M6809_INLINE void lbvc(void);
M6809_INLINE void lbvs(void);
M6809_INLINE void lda_di(void);
M6809_INLINE void lda_ex(void);
M6809_INLINE void lda_im(void);
M6809_INLINE void lda_ix(void);
M6809_INLINE void ldb_di(void);
M6809_INLINE void ldb_ex(void);
M6809_INLINE void ldb_im(void);
M6809_INLINE void ldb_ix(void);
M6809_INLINE void ldd_di(void);
M6809_INLINE void ldd_ex(void);
M6809_INLINE void ldd_im(void);
M6809_INLINE void ldd_ix(void);
M6809_INLINE void lds_di(void);
M6809_INLINE void lds_ex(void);
M6809_INLINE void lds_im(void);
M6809_INLINE void lds_ix(void);
M6809_INLINE void ldu_di(void);
M6809_INLINE void ldu_ex(void);
M6809_INLINE void ldu_im(void);
M6809_INLINE void ldu_ix(void);
M6809_INLINE void ldx_di(void);
M6809_INLINE void ldx_ex(void);
M6809_INLINE void ldx_im(void);
M6809_INLINE void ldx_ix(void);
M6809_INLINE void ldy_di(void);
M6809_INLINE void ldy_ex(void);
M6809_INLINE void ldy_im(void);
M6809_INLINE void ldy_ix(void);
M6809_INLINE void leas(void);
M6809_INLINE void leau(void);
M6809_INLINE void leax(void);
M6809_INLINE void leay(void);
M6809_INLINE void lsr_di(void);
M6809_INLINE void lsr_ex(void);
M6809_INLINE void lsr_ix(void);
M6809_INLINE void lsra(void);
M6809_INLINE void lsrb(void);
M6809_INLINE void mul(void);
M6809_INLINE void neg_di(void);
M6809_INLINE void neg_ex(void);
M6809_INLINE void neg_ix(void);
M6809_INLINE void nega(void);
M6809_INLINE void negb(void);
M6809_INLINE void nop(void);
M6809_INLINE void ora_di(void);
M6809_INLINE void ora_ex(void);
M6809_INLINE void ora_im(void);
M6809_INLINE void ora_ix(void);
M6809_INLINE void orb_di(void);
M6809_INLINE void orb_ex(void);
M6809_INLINE void orb_im(void);
M6809_INLINE void orb_ix(void);
M6809_INLINE void orcc(void);
M6809_INLINE void pshs(void);
M6809_INLINE void pshu(void);
M6809_INLINE void puls(void);
M6809_INLINE void pulu(void);
M6809_INLINE void rol_di(void);
M6809_INLINE void rol_ex(void);
M6809_INLINE void rol_ix(void);
M6809_INLINE void rola(void);
M6809_INLINE void rolb(void);
M6809_INLINE void ror_di(void);
M6809_INLINE void ror_ex(void);
M6809_INLINE void ror_ix(void);
M6809_INLINE void rora(void);
M6809_INLINE void rorb(void);
M6809_INLINE void rti(void);
M6809_INLINE void rts(void);
M6809_INLINE void sbca_di(void);
M6809_INLINE void sbca_ex(void);
M6809_INLINE void sbca_im(void);
M6809_INLINE void sbca_ix(void);
M6809_INLINE void sbcb_di(void);
M6809_INLINE void sbcb_ex(void);
M6809_INLINE void sbcb_im(void);
M6809_INLINE void sbcb_ix(void);
M6809_INLINE void sex(void);
M6809_INLINE void sta_di(void);
M6809_INLINE void sta_ex(void);
M6809_INLINE void sta_im(void);
M6809_INLINE void sta_ix(void);
M6809_INLINE void stb_di(void);
M6809_INLINE void stb_ex(void);
M6809_INLINE void stb_im(void);
M6809_INLINE void stb_ix(void);
M6809_INLINE void std_di(void);
M6809_INLINE void std_ex(void);
M6809_INLINE void std_im(void);
M6809_INLINE void std_ix(void);
M6809_INLINE void sts_di(void);
M6809_INLINE void sts_ex(void);
M6809_INLINE void sts_im(void);
M6809_INLINE void sts_ix(void);
M6809_INLINE void stu_di(void);
M6809_INLINE void stu_ex(void);
M6809_INLINE void stu_im(void);
M6809_INLINE void stu_ix(void);
M6809_INLINE void stx_di(void);
M6809_INLINE void stx_ex(void);
M6809_INLINE void stx_im(void);
M6809_INLINE void stx_ix(void);
M6809_INLINE void sty_di(void);
M6809_INLINE void sty_ex(void);
M6809_INLINE void sty_im(void);
M6809_INLINE void sty_ix(void);
M6809_INLINE void suba_di(void);
M6809_INLINE void suba_ex(void);
M6809_INLINE void suba_im(void);
M6809_INLINE void suba_ix(void);
M6809_INLINE void subb_di(void);
M6809_INLINE void subb_ex(void);
M6809_INLINE void subb_im(void);
M6809_INLINE void subb_ix(void);
M6809_INLINE void subd_di(void);
M6809_INLINE void subd_ex(void);
M6809_INLINE void subd_im(void);
M6809_INLINE void subd_ix(void);
M6809_INLINE void swi(void);
M6809_INLINE void swi2(void);
M6809_INLINE void swi3(void);
M6809_INLINE void sync(void);
M6809_INLINE void tfr(void);
M6809_INLINE void tst_di(void);
M6809_INLINE void tst_ex(void);
M6809_INLINE void tst_ix(void);
M6809_INLINE void tsta(void);
M6809_INLINE void tstb(void);

M6809_INLINE void pref10(void);
M6809_INLINE void pref11(void);

#if (BIG_SWITCH==0)
static void (*const m6809_main[0x100])(void) = {
	neg_di, neg_di, illegal,com_di, lsr_di, illegal,ror_di, asr_di, 	/* 00 */
	asl_di, rol_di, dec_di, illegal,inc_di, tst_di, jmp_di, clr_di,
	pref10, pref11, nop,	sync,	illegal,illegal,lbra,	lbsr,		/* 10 */
	illegal,daa,	orcc,	illegal,andcc,	sex,	exg,	tfr,
	bra,	brn,	bhi,	bls,	bcc,	bcs,	bne,	beq,		/* 20 */
	bvc,	bvs,	bpl,	bmi,	bge,	blt,	bgt,	ble,
	leax,	leay,	leas,	leau,	pshs,	puls,	pshu,	pulu,		/* 30 */
	illegal,rts,	abx,	rti,	cwai,	mul,	illegal,swi,
	nega,	illegal,illegal,coma,	lsra,	illegal,rora,	asra,		/* 40 */
	asla,	rola,	deca,	illegal,inca,	tsta,	illegal,clra,
	negb,	illegal,illegal,comb,	lsrb,	illegal,rorb,	asrb,		/* 50 */
	aslb,	rolb,	decb,	illegal,incb,	tstb,	illegal,clrb,
	neg_ix, illegal,illegal,com_ix, lsr_ix, illegal,ror_ix, asr_ix, 	/* 60 */
	asl_ix, rol_ix, dec_ix, illegal,inc_ix, tst_ix, jmp_ix, clr_ix,
	neg_ex, illegal,illegal,com_ex, lsr_ex, illegal,ror_ex, asr_ex, 	/* 70 */
	asl_ex, rol_ex, dec_ex, illegal,inc_ex, tst_ex, jmp_ex, clr_ex,
	suba_im,cmpa_im,sbca_im,subd_im,anda_im,bita_im,lda_im, sta_im, 	/* 80 */
	eora_im,adca_im,ora_im, adda_im,cmpx_im,bsr,	ldx_im, stx_im,
	suba_di,cmpa_di,sbca_di,subd_di,anda_di,bita_di,lda_di, sta_di, 	/* 90 */
	eora_di,adca_di,ora_di, adda_di,cmpx_di,jsr_di, ldx_di, stx_di,
	suba_ix,cmpa_ix,sbca_ix,subd_ix,anda_ix,bita_ix,lda_ix, sta_ix, 	/* a0 */
	eora_ix,adca_ix,ora_ix, adda_ix,cmpx_ix,jsr_ix, ldx_ix, stx_ix,
	suba_ex,cmpa_ex,sbca_ex,subd_ex,anda_ex,bita_ex,lda_ex, sta_ex, 	/* b0 */
	eora_ex,adca_ex,ora_ex, adda_ex,cmpx_ex,jsr_ex, ldx_ex, stx_ex,
	subb_im,cmpb_im,sbcb_im,addd_im,andb_im,bitb_im,ldb_im, stb_im, 	/* c0 */
	eorb_im,adcb_im,orb_im, addb_im,ldd_im, std_im, ldu_im, stu_im,
	subb_di,cmpb_di,sbcb_di,addd_di,andb_di,bitb_di,ldb_di, stb_di, 	/* d0 */
	eorb_di,adcb_di,orb_di, addb_di,ldd_di, std_di, ldu_di, stu_di,
	subb_ix,cmpb_ix,sbcb_ix,addd_ix,andb_ix,bitb_ix,ldb_ix, stb_ix, 	/* e0 */
	eorb_ix,adcb_ix,orb_ix, addb_ix,ldd_ix, std_ix, ldu_ix, stu_ix,
	subb_ex,cmpb_ex,sbcb_ex,addd_ex,andb_ex,bitb_ex,ldb_ex, stb_ex, 	/* f0 */
	eorb_ex,adcb_ex,orb_ex, addb_ex,ldd_ex, std_ex, ldu_ex, stu_ex
};
#endif

/*

HNZVC

? = undefined
* = affected
- = unaffected
0 = cleared
1 = set
# = CCr directly affected by instruction
@ = special - carry set if bit 7 is set

*/

#ifdef NEW
static void illegal( void )
#else
M6809_INLINE void illegal( void )
#endif
{
//	logerror("M6809: illegal opcode at %04x\n",PC);
}

/* $00 NEG direct ?**** */
M6809_INLINE void neg_di( void )
{
	UINT16 r,t;
	DIRBYTE(t);
	r = -t;
	CLR_NZVC;
	SET_FLAGS8(0,t,r);
	WM(EAD,r);
}

/* $01 ILLEGAL */

/* $02 ILLEGAL */

/* $03 COM direct -**01 */
M6809_INLINE void com_di( void )
{
	UINT8 t;
	DIRBYTE(t);
	t = ~t;
	CLR_NZV;
	SET_NZ8(t);
	SEC;
	WM(EAD,t);
}

/* $04 LSR direct -0*-* */
M6809_INLINE void lsr_di( void )
{
	UINT8 t;
	DIRBYTE(t);
	CLR_NZC;
	CC |= (t & CC_C);
	t >>= 1;
	SET_Z8(t);
	WM(EAD,t);
}

/* $05 ILLEGAL */

/* $06 ROR direct -**-* */
M6809_INLINE void ror_di( void )
{
	UINT8 t,r;
	DIRBYTE(t);
	r= (CC & CC_C) << 7;
	CLR_NZC;
	CC |= (t & CC_C);
	r |= t>>1;
	SET_NZ8(r);
	WM(EAD,r);
}

/* $07 ASR direct ?**-* */
M6809_INLINE void asr_di( void )
{
	UINT8 t;
	DIRBYTE(t);
	CLR_NZC;
	CC |= (t & CC_C);
	t = (t & 0x80) | (t >> 1);
	SET_NZ8(t);
	WM(EAD,t);
}

/* $08 ASL direct ?**** */
M6809_INLINE void asl_di( void )
{
	UINT16 t,r;
	DIRBYTE(t);
	r = t << 1;
	CLR_NZVC;
	SET_FLAGS8(t,t,r);
	WM(EAD,r);
}

/* $09 ROL direct -**** */
M6809_INLINE void rol_di( void )
{
	UINT16 t,r;
	DIRBYTE(t);
	r = (CC & CC_C) | (t << 1);
	CLR_NZVC;
	SET_FLAGS8(t,t,r);
	WM(EAD,r);
}

/* $0A DEC direct -***- */
M6809_INLINE void dec_di( void )
{
	UINT8 t;
	DIRBYTE(t);
	--t;
	CLR_NZV;
	SET_FLAGS8D(t);
	WM(EAD,t);
}

/* $0B ILLEGAL */

/* $OC INC direct -***- */
M6809_INLINE void inc_di( void )
{
	UINT8 t;
	DIRBYTE(t);
	++t;
	CLR_NZV;
	SET_FLAGS8I(t);
	WM(EAD,t);
}

/* $OD TST direct -**0- */
M6809_INLINE void tst_di( void )
{
	UINT8 t;
	DIRBYTE(t);
	CLR_NZV;
	SET_NZ8(t);
}

/* $0E JMP direct ----- */
M6809_INLINE void jmp_di( void )
{
	DIRECT;
	PCD = EAD;
	CHANGE_PC;
}

/* $0F CLR direct -0100 */
M6809_INLINE void clr_di( void )
{
	DIRECT;
	(void)RM(EAD);
	WM(EAD,0);
	CLR_NZVC;
	SEZ;
}

/* $10 FLAG */

/* $11 FLAG */

/* $12 NOP inherent ----- */
M6809_INLINE void nop( void )
{
	;
}

/* $13 SYNC inherent ----- */
M6809_INLINE void sync( void )
{
	/* SYNC stops processing instructions until an interrupt request happens. */
	/* This doesn't require the corresponding interrupt to be enabled: if it */
	/* is disabled, execution continues with the next instruction. */
	m6809.int_state |= M6809_SYNC;	 /* HJB 990227 */
	CHECK_IRQ_LINES;
	/* if M6809_SYNC has not been cleared by CHECK_IRQ_LINES,
     * stop execution until the interrupt lines change. */
	if( m6809.int_state & M6809_SYNC )
		if (m6809_ICount > 0) m6809_ICount = 0;
}

/* $14 ILLEGAL */

/* $15 ILLEGAL */

/* $16 LBRA relative ----- */
M6809_INLINE void lbra( void )
{
	IMMWORD(ea);
	PC += EA;
	CHANGE_PC;

	if ( EA == 0xfffd )  /* EHC 980508 speed up busy loop */
		if ( m6809_ICount > 0)
			m6809_ICount = 0;
}

/* $17 LBSR relative ----- */
M6809_INLINE void lbsr( void )
{
	IMMWORD(ea);
	PUSHWORD(pPC);
	PC += EA;
	CHANGE_PC;
}

/* $18 ILLEGAL */

/* $19 DAA inherent (A) -**0* */
M6809_INLINE void daa( void )
{
	UINT8 msn, lsn;
	UINT16 t, cf = 0;
	msn = A & 0xf0; lsn = A & 0x0f;
	if( lsn>0x09 || CC & CC_H) cf |= 0x06;
	if( msn>0x80 && lsn>0x09 ) cf |= 0x60;
	if( msn>0x90 || CC & CC_C) cf |= 0x60;
	t = cf + A;
	CLR_NZV; /* keep carry from previous operation */
	SET_NZ8((UINT8)t); SET_C8(t);
	A = t;
}

/* $1A ORCC immediate ##### */
M6809_INLINE void orcc( void )
{
	UINT8 t;
	IMMBYTE(t);
	CC |= t;
	CHECK_IRQ_LINES;	/* HJB 990116 */
}

/* $1B ILLEGAL */

/* $1C ANDCC immediate ##### */
M6809_INLINE void andcc( void )
{
	UINT8 t;
	IMMBYTE(t);
	CC &= t;
	CHECK_IRQ_LINES;	/* HJB 990116 */
}

/* $1D SEX inherent -**-- */
M6809_INLINE void sex( void )
{
	UINT16 t;
	t = SIGNED(B);
	D = t;
//  CLR_NZV;    Tim Lindner 20020905: verified that V flag is not affected
	CLR_NZ;
	SET_NZ16(t);
}

/* $1E EXG inherent ----- */
M6809_INLINE void exg( void )
{
	UINT16 t1,t2;
	UINT8 tb;

	IMMBYTE(tb);
	if( (tb^(tb>>4)) & 0x08 )	/* HJB 990225: mixed 8/16 bit case? */
	{
		/* transfer $ff to both registers */
		t1 = t2 = 0xff;
	}
	else
	{
		switch(tb>>4) {
			case  0: t1 = D;  break;
			case  1: t1 = X;  break;
			case  2: t1 = Y;  break;
			case  3: t1 = U;  break;
			case  4: t1 = S;  break;
			case  5: t1 = PC; break;
			case  8: t1 = A;  break;
			case  9: t1 = B;  break;
			case 10: t1 = CC; break;
			case 11: t1 = DP; break;
			default: t1 = 0xff;
		}
		switch(tb&15) {
			case  0: t2 = D;  break;
			case  1: t2 = X;  break;
			case  2: t2 = Y;  break;
			case  3: t2 = U;  break;
			case  4: t2 = S;  break;
			case  5: t2 = PC; break;
			case  8: t2 = A;  break;
			case  9: t2 = B;  break;
			case 10: t2 = CC; break;
			case 11: t2 = DP; break;
			default: t2 = 0xff;
        }
	}
	switch(tb>>4) {
		case  0: D = t2;  break;
		case  1: X = t2;  break;
		case  2: Y = t2;  break;
		case  3: U = t2;  break;
		case  4: S = t2;  break;
		case  5: PC = t2; CHANGE_PC; break;
		case  8: A = t2;  break;
		case  9: B = t2;  break;
		case 10: CC = t2; break;
		case 11: DP = t2; break;
	}
	switch(tb&15) {
		case  0: D = t1;  break;
		case  1: X = t1;  break;
		case  2: Y = t1;  break;
		case  3: U = t1;  break;
		case  4: S = t1;  break;
		case  5: PC = t1; CHANGE_PC; break;
		case  8: A = t1;  break;
		case  9: B = t1;  break;
		case 10: CC = t1; break;
		case 11: DP = t1; break;
	}
}

/* $1F TFR inherent ----- */
M6809_INLINE void tfr( void )
{
	UINT8 tb;
	UINT16 t;

	IMMBYTE(tb);
	if( (tb^(tb>>4)) & 0x08 )	/* HJB 990225: mixed 8/16 bit case? */
	{
		/* transfer $ff to register */
		t = 0xff;
    }
	else
	{
		switch(tb>>4) {
			case  0: t = D;  break;
			case  1: t = X;  break;
			case  2: t = Y;  break;
			case  3: t = U;  break;
			case  4: t = S;  break;
			case  5: t = PC; break;
			case  8: t = A;  break;
			case  9: t = B;  break;
			case 10: t = CC; break;
			case 11: t = DP; break;
			default: t = 0xff;
        }
	}
	switch(tb&15) {
		case  0: D = t;  break;
		case  1: X = t;  break;
		case  2: Y = t;  break;
		case  3: U = t;  break;
		case  4: S = t;  break;
		case  5: PC = t; CHANGE_PC; break;
		case  8: A = t;  break;
		case  9: B = t;  break;
		case 10: CC = t; break;
		case 11: DP = t; break;
    }
}

/* $20 BRA relative ----- */
M6809_INLINE void bra( void )
{
	UINT8 t;
	IMMBYTE(t);
	PC += SIGNED(t);
    CHANGE_PC;
	/* JB 970823 - speed up busy loops */
	if( t == 0xfe )
		if( m6809_ICount > 0 ) m6809_ICount = 0;
}

/* $21 BRN relative ----- */
M6809_INLINE void brn( void )
{
	UINT8 t;
	IMMBYTE(t);
}

/* $1021 LBRN relative ----- */
M6809_INLINE void lbrn( void )
{
	IMMWORD(ea);
}

/* $22 BHI relative ----- */
M6809_INLINE void bhi( void )
{
	BRANCH( !(CC & (CC_Z|CC_C)) );
}

/* $1022 LBHI relative ----- */
M6809_INLINE void lbhi( void )
{
	LBRANCH( !(CC & (CC_Z|CC_C)) );
}

/* $23 BLS relative ----- */
M6809_INLINE void bls( void )
{
	BRANCH( (CC & (CC_Z|CC_C)) );
}

/* $1023 LBLS relative ----- */
M6809_INLINE void lbls( void )
{
	LBRANCH( (CC&(CC_Z|CC_C)) );
}

/* $24 BCC relative ----- */
M6809_INLINE void bcc( void )
{
	BRANCH( !(CC&CC_C) );
}

/* $1024 LBCC relative ----- */
M6809_INLINE void lbcc( void )
{
	LBRANCH( !(CC&CC_C) );
}

/* $25 BCS relative ----- */
M6809_INLINE void bcs( void )
{
	BRANCH( (CC&CC_C) );
}

/* $1025 LBCS relative ----- */
M6809_INLINE void lbcs( void )
{
	LBRANCH( (CC&CC_C) );
}

/* $26 BNE relative ----- */
M6809_INLINE void bne( void )
{
	BRANCH( !(CC&CC_Z) );
}

/* $1026 LBNE relative ----- */
M6809_INLINE void lbne( void )
{
	LBRANCH( !(CC&CC_Z) );
}

/* $27 BEQ relative ----- */
M6809_INLINE void beq( void )
{
	BRANCH( (CC&CC_Z) );
}

/* $1027 LBEQ relative ----- */
M6809_INLINE void lbeq( void )
{
	LBRANCH( (CC&CC_Z) );
}

/* $28 BVC relative ----- */
M6809_INLINE void bvc( void )
{
	BRANCH( !(CC&CC_V) );
}

/* $1028 LBVC relative ----- */
M6809_INLINE void lbvc( void )
{
	LBRANCH( !(CC&CC_V) );
}

/* $29 BVS relative ----- */
M6809_INLINE void bvs( void )
{
	BRANCH( (CC&CC_V) );
}

/* $1029 LBVS relative ----- */
M6809_INLINE void lbvs( void )
{
	LBRANCH( (CC&CC_V) );
}

/* $2A BPL relative ----- */
M6809_INLINE void bpl( void )
{
	BRANCH( !(CC&CC_N) );
}

/* $102A LBPL relative ----- */
M6809_INLINE void lbpl( void )
{
	LBRANCH( !(CC&CC_N) );
}

/* $2B BMI relative ----- */
M6809_INLINE void bmi( void )
{
	BRANCH( (CC&CC_N) );
}

/* $102B LBMI relative ----- */
M6809_INLINE void lbmi( void )
{
	LBRANCH( (CC&CC_N) );
}

/* $2C BGE relative ----- */
M6809_INLINE void bge( void )
{
	BRANCH( !NXORV );
}

/* $102C LBGE relative ----- */
M6809_INLINE void lbge( void )
{
	LBRANCH( !NXORV );
}

/* $2D BLT relative ----- */
M6809_INLINE void blt( void )
{
	BRANCH( NXORV );
}

/* $102D LBLT relative ----- */
M6809_INLINE void lblt( void )
{
	LBRANCH( NXORV );
}

/* $2E BGT relative ----- */
M6809_INLINE void bgt( void )
{
	BRANCH( !(NXORV || (CC&CC_Z)) );
}

/* $102E LBGT relative ----- */
M6809_INLINE void lbgt( void )
{
	LBRANCH( !(NXORV || (CC&CC_Z)) );
}

/* $2F BLE relative ----- */
M6809_INLINE void ble( void )
{
	BRANCH( (NXORV || (CC&CC_Z)) );
}

/* $102F LBLE relative ----- */
M6809_INLINE void lble( void )
{
	LBRANCH( (NXORV || (CC&CC_Z)) );
}

/* $30 LEAX indexed --*-- */
M6809_INLINE void leax( void )
{
	fetch_effective_address();
    X = EA;
	CLR_Z;
	SET_Z(X);
}

/* $31 LEAY indexed --*-- */
M6809_INLINE void leay( void )
{
	fetch_effective_address();
    Y = EA;
	CLR_Z;
	SET_Z(Y);
}

/* $32 LEAS indexed ----- */
M6809_INLINE void leas( void )
{
	fetch_effective_address();
    S = EA;
	m6809.int_state |= M6809_LDS;
}

/* $33 LEAU indexed ----- */
M6809_INLINE void leau( void )
{
	fetch_effective_address();
    U = EA;
}

/* $34 PSHS inherent ----- */
M6809_INLINE void pshs( void )
{
	UINT8 t;
	IMMBYTE(t);
	if( t&0x80 ) { PUSHWORD(pPC); m6809_ICount -= 2; }
	if( t&0x40 ) { PUSHWORD(pU);  m6809_ICount -= 2; }
	if( t&0x20 ) { PUSHWORD(pY);  m6809_ICount -= 2; }
	if( t&0x10 ) { PUSHWORD(pX);  m6809_ICount -= 2; }
	if( t&0x08 ) { PUSHBYTE(DP);  m6809_ICount -= 1; }
	if( t&0x04 ) { PUSHBYTE(B);   m6809_ICount -= 1; }
	if( t&0x02 ) { PUSHBYTE(A);   m6809_ICount -= 1; }
	if( t&0x01 ) { PUSHBYTE(CC);  m6809_ICount -= 1; }
}

/* 35 PULS inherent ----- */
M6809_INLINE void puls( void )
{
	UINT8 t;
	IMMBYTE(t);
	if( t&0x01 ) { PULLBYTE(CC); m6809_ICount -= 1; }
	if( t&0x02 ) { PULLBYTE(A);  m6809_ICount -= 1; }
	if( t&0x04 ) { PULLBYTE(B);  m6809_ICount -= 1; }
	if( t&0x08 ) { PULLBYTE(DP); m6809_ICount -= 1; }
	if( t&0x10 ) { PULLWORD(XD); m6809_ICount -= 2; }
	if( t&0x20 ) { PULLWORD(YD); m6809_ICount -= 2; }
	if( t&0x40 ) { PULLWORD(UD); m6809_ICount -= 2; }
	if( t&0x80 ) { PULLWORD(PCD); CHANGE_PC; m6809_ICount -= 2; }

	/* HJB 990225: moved check after all PULLs */
	if( t&0x01 ) { CHECK_IRQ_LINES; }
}

/* $36 PSHU inherent ----- */
M6809_INLINE void pshu( void )
{
	UINT8 t;
	IMMBYTE(t);
	if( t&0x80 ) { PSHUWORD(pPC); m6809_ICount -= 2; }
	if( t&0x40 ) { PSHUWORD(pS);  m6809_ICount -= 2; }
	if( t&0x20 ) { PSHUWORD(pY);  m6809_ICount -= 2; }
	if( t&0x10 ) { PSHUWORD(pX);  m6809_ICount -= 2; }
	if( t&0x08 ) { PSHUBYTE(DP);  m6809_ICount -= 1; }
	if( t&0x04 ) { PSHUBYTE(B);   m6809_ICount -= 1; }
	if( t&0x02 ) { PSHUBYTE(A);   m6809_ICount -= 1; }
	if( t&0x01 ) { PSHUBYTE(CC);  m6809_ICount -= 1; }
}

/* 37 PULU inherent ----- */
M6809_INLINE void pulu( void )
{
	UINT8 t;
	IMMBYTE(t);
	if( t&0x01 ) { PULUBYTE(CC); m6809_ICount -= 1; }
	if( t&0x02 ) { PULUBYTE(A);  m6809_ICount -= 1; }
	if( t&0x04 ) { PULUBYTE(B);  m6809_ICount -= 1; }
	if( t&0x08 ) { PULUBYTE(DP); m6809_ICount -= 1; }
	if( t&0x10 ) { PULUWORD(XD); m6809_ICount -= 2; }
	if( t&0x20 ) { PULUWORD(YD); m6809_ICount -= 2; }
	if( t&0x40 ) { PULUWORD(SD); m6809_ICount -= 2; }
	if( t&0x80 ) { PULUWORD(PCD); CHANGE_PC; m6809_ICount -= 2; }

	/* HJB 990225: moved check after all PULLs */
	if( t&0x01 ) { CHECK_IRQ_LINES; }
}

/* $38 ILLEGAL */

/* $39 RTS inherent ----- */
M6809_INLINE void rts( void )
{
	PULLWORD(PCD);
	CHANGE_PC;
}

/* $3A ABX inherent ----- */
M6809_INLINE void abx( void )
{
	X += B;
}

/* $3B RTI inherent ##### */
M6809_INLINE void rti( void )
{
	UINT8 t;
	PULLBYTE(CC);
	t = CC & CC_E;		/* HJB 990225: entire state saved? */
	if(t)
	{
        m6809_ICount -= 9;
		PULLBYTE(A);
		PULLBYTE(B);
		PULLBYTE(DP);
		PULLWORD(XD);
		PULLWORD(YD);
		PULLWORD(UD);
	}
	PULLWORD(PCD);
	CHANGE_PC;
	CHECK_IRQ_LINES;	/* HJB 990116 */
}

/* $3C CWAI inherent ----1 */
M6809_INLINE void cwai( void )
{
	UINT8 t;
	IMMBYTE(t);
	CC &= t;
	/*
     * CWAI stacks the entire machine state on the hardware stack,
     * then waits for an interrupt; when the interrupt is taken
     * later, the state is *not* saved again after CWAI.
     */
	CC |= CC_E; 		/* HJB 990225: save entire state */
	PUSHWORD(pPC);
	PUSHWORD(pU);
	PUSHWORD(pY);
	PUSHWORD(pX);
	PUSHBYTE(DP);
	PUSHBYTE(B);
	PUSHBYTE(A);
	PUSHBYTE(CC);
	m6809.int_state |= M6809_CWAI;	 /* HJB 990228 */
    CHECK_IRQ_LINES;    /* HJB 990116 */
	if( m6809.int_state & M6809_CWAI )
		if( m6809_ICount > 0 )
			m6809_ICount = 0;
}

/* $3D MUL inherent --*-@ */
M6809_INLINE void mul( void )
{
	UINT16 t;
	t = A * B;
	CLR_ZC; SET_Z16(t); if(t&0x80) SEC;
	D = t;
}

/* $3E ILLEGAL */

/* $3F SWI (SWI2 SWI3) absolute indirect ----- */
M6809_INLINE void swi( void )
{
	CC |= CC_E; 			/* HJB 980225: save entire state */
	PUSHWORD(pPC);
	PUSHWORD(pU);
	PUSHWORD(pY);
	PUSHWORD(pX);
	PUSHBYTE(DP);
	PUSHBYTE(B);
	PUSHBYTE(A);
	PUSHBYTE(CC);
	CC |= CC_IF | CC_II;	/* inhibit FIRQ and IRQ */
	PCD=RM16(0xfffa);
	CHANGE_PC;
}

/* $103F SWI2 absolute indirect ----- */
M6809_INLINE void swi2( void )
{
	CC |= CC_E; 			/* HJB 980225: save entire state */
	PUSHWORD(pPC);
	PUSHWORD(pU);
	PUSHWORD(pY);
	PUSHWORD(pX);
	PUSHBYTE(DP);
	PUSHBYTE(B);
	PUSHBYTE(A);
    PUSHBYTE(CC);
	PCD = RM16(0xfff4);
	CHANGE_PC;
}

/* $113F SWI3 absolute indirect ----- */
M6809_INLINE void swi3( void )
{
	CC |= CC_E; 			/* HJB 980225: save entire state */
	PUSHWORD(pPC);
	PUSHWORD(pU);
	PUSHWORD(pY);
	PUSHWORD(pX);
	PUSHBYTE(DP);
	PUSHBYTE(B);
	PUSHBYTE(A);
    PUSHBYTE(CC);
	PCD = RM16(0xfff2);
	CHANGE_PC;
}

/* $40 NEGA inherent ?**** */
M6809_INLINE void nega( void )
{
	UINT16 r;
	r = -A;
	CLR_NZVC;
	SET_FLAGS8(0,A,r);
	A = r;
}

/* $41 ILLEGAL */

/* $42 ILLEGAL */

/* $43 COMA inherent -**01 */
M6809_INLINE void coma( void )
{
	A = ~A;
	CLR_NZV;
	SET_NZ8(A);
	SEC;
}

/* $44 LSRA inherent -0*-* */
M6809_INLINE void lsra( void )
{
	CLR_NZC;
	CC |= (A & CC_C);
	A >>= 1;
	SET_Z8(A);
}

/* $45 ILLEGAL */

/* $46 RORA inherent -**-* */
M6809_INLINE void rora( void )
{
	UINT8 r;
	r = (CC & CC_C) << 7;
	CLR_NZC;
	CC |= (A & CC_C);
	r |= A >> 1;
	SET_NZ8(r);
	A = r;
}

/* $47 ASRA inherent ?**-* */
M6809_INLINE void asra( void )
{
	CLR_NZC;
	CC |= (A & CC_C);
	A = (A & 0x80) | (A >> 1);
	SET_NZ8(A);
}

/* $48 ASLA inherent ?**** */
M6809_INLINE void asla( void )
{
	UINT16 r;
	r = A << 1;
	CLR_NZVC;
	SET_FLAGS8(A,A,r);
	A = r;
}

/* $49 ROLA inherent -**** */
M6809_INLINE void rola( void )
{
	UINT16 t,r;
	t = A;
	r = (CC & CC_C) | (t<<1);
	CLR_NZVC; SET_FLAGS8(t,t,r);
	A = r;
}

/* $4A DECA inherent -***- */
M6809_INLINE void deca( void )
{
	--A;
	CLR_NZV;
	SET_FLAGS8D(A);
}

/* $4B ILLEGAL */

/* $4C INCA inherent -***- */
M6809_INLINE void inca( void )
{
	++A;
	CLR_NZV;
	SET_FLAGS8I(A);
}

/* $4D TSTA inherent -**0- */
M6809_INLINE void tsta( void )
{
	CLR_NZV;
	SET_NZ8(A);
}

/* $4E ILLEGAL */

/* $4F CLRA inherent -0100 */
M6809_INLINE void clra( void )
{
	A = 0;
	CLR_NZVC; SEZ;
}

/* $50 NEGB inherent ?**** */
M6809_INLINE void negb( void )
{
	UINT16 r;
	r = -B;
	CLR_NZVC;
	SET_FLAGS8(0,B,r);
	B = r;
}

/* $51 ILLEGAL */

/* $52 ILLEGAL */

/* $53 COMB inherent -**01 */
M6809_INLINE void comb( void )
{
	B = ~B;
	CLR_NZV;
	SET_NZ8(B);
	SEC;
}

/* $54 LSRB inherent -0*-* */
M6809_INLINE void lsrb( void )
{
	CLR_NZC;
	CC |= (B & CC_C);
	B >>= 1;
	SET_Z8(B);
}

/* $55 ILLEGAL */

/* $56 RORB inherent -**-* */
M6809_INLINE void rorb( void )
{
	UINT8 r;
	r = (CC & CC_C) << 7;
	CLR_NZC;
	CC |= (B & CC_C);
	r |= B >> 1;
	SET_NZ8(r);
	B = r;
}

/* $57 ASRB inherent ?**-* */
M6809_INLINE void asrb( void )
{
	CLR_NZC;
	CC |= (B & CC_C);
	B= (B & 0x80) | (B >> 1);
	SET_NZ8(B);
}

/* $58 ASLB inherent ?**** */
M6809_INLINE void aslb( void )
{
	UINT16 r;
	r = B << 1;
	CLR_NZVC;
	SET_FLAGS8(B,B,r);
	B = r;
}

/* $59 ROLB inherent -**** */
M6809_INLINE void rolb( void )
{
	UINT16 t,r;
	t = B;
	r = CC & CC_C;
	r |= t << 1;
	CLR_NZVC;
	SET_FLAGS8(t,t,r);
	B = r;
}

/* $5A DECB inherent -***- */
M6809_INLINE void decb( void )
{
	--B;
	CLR_NZV;
	SET_FLAGS8D(B);
}

/* $5B ILLEGAL */

/* $5C INCB inherent -***- */
M6809_INLINE void incb( void )
{
	++B;
	CLR_NZV;
	SET_FLAGS8I(B);
}

/* $5D TSTB inherent -**0- */
M6809_INLINE void tstb( void )
{
	CLR_NZV;
	SET_NZ8(B);
}

/* $5E ILLEGAL */

/* $5F CLRB inherent -0100 */
M6809_INLINE void clrb( void )
{
	B = 0;
	CLR_NZVC; SEZ;
}

/* $60 NEG indexed ?**** */
M6809_INLINE void neg_ix( void )
{
	UINT16 r,t;
	fetch_effective_address();
	t = RM(EAD);
	r=-t;
	CLR_NZVC;
	SET_FLAGS8(0,t,r);
	WM(EAD,r);
}

/* $61 ILLEGAL */

/* $62 ILLEGAL */

/* $63 COM indexed -**01 */
M6809_INLINE void com_ix( void )
{
	UINT8 t;
	fetch_effective_address();
	t = ~RM(EAD);
	CLR_NZV;
	SET_NZ8(t);
	SEC;
	WM(EAD,t);
}

/* $64 LSR indexed -0*-* */
M6809_INLINE void lsr_ix( void )
{
	UINT8 t;
	fetch_effective_address();
	t=RM(EAD);
	CLR_NZC;
	CC |= (t & CC_C);
	t>>=1; SET_Z8(t);
	WM(EAD,t);
}

/* $65 ILLEGAL */

/* $66 ROR indexed -**-* */
M6809_INLINE void ror_ix( void )
{
	UINT8 t,r;
	fetch_effective_address();
	t=RM(EAD);
	r = (CC & CC_C) << 7;
	CLR_NZC;
	CC |= (t & CC_C);
	r |= t>>1; SET_NZ8(r);
	WM(EAD,r);
}

/* $67 ASR indexed ?**-* */
M6809_INLINE void asr_ix( void )
{
	UINT8 t;
	fetch_effective_address();
	t=RM(EAD);
	CLR_NZC;
	CC |= (t & CC_C);
	t=(t&0x80)|(t>>1);
	SET_NZ8(t);
	WM(EAD,t);
}

/* $68 ASL indexed ?**** */
M6809_INLINE void asl_ix( void )
{
	UINT16 t,r;
	fetch_effective_address();
	t=RM(EAD);
	r = t << 1;
	CLR_NZVC;
	SET_FLAGS8(t,t,r);
	WM(EAD,r);
}

/* $69 ROL indexed -**** */
M6809_INLINE void rol_ix( void )
{
	UINT16 t,r;
	fetch_effective_address();
	t=RM(EAD);
	r = CC & CC_C;
	r |= t << 1;
	CLR_NZVC;
	SET_FLAGS8(t,t,r);
	WM(EAD,r);
}

/* $6A DEC indexed -***- */
M6809_INLINE void dec_ix( void )
{
	UINT8 t;
	fetch_effective_address();
	t = RM(EAD) - 1;
	CLR_NZV; SET_FLAGS8D(t);
	WM(EAD,t);
}

/* $6B ILLEGAL */

/* $6C INC indexed -***- */
M6809_INLINE void inc_ix( void )
{
	UINT8 t;
	fetch_effective_address();
	t = RM(EAD) + 1;
	CLR_NZV; SET_FLAGS8I(t);
	WM(EAD,t);
}

/* $6D TST indexed -**0- */
M6809_INLINE void tst_ix( void )
{
	UINT8 t;
	fetch_effective_address();
	t = RM(EAD);
	CLR_NZV;
	SET_NZ8(t);
}

/* $6E JMP indexed ----- */
M6809_INLINE void jmp_ix( void )
{
	fetch_effective_address();
	PCD = EAD;
	CHANGE_PC;
}

/* $6F CLR indexed -0100 */
M6809_INLINE void clr_ix( void )
{
	fetch_effective_address();
	(void)RM(EAD);
	WM(EAD,0);
	CLR_NZVC; SEZ;
}

/* $70 NEG extended ?**** */
M6809_INLINE void neg_ex( void )
{
	UINT16 r,t;
	EXTBYTE(t); r=-t;
	CLR_NZVC; SET_FLAGS8(0,t,r);
	WM(EAD,r);
}

/* $71 ILLEGAL */

/* $72 ILLEGAL */

/* $73 COM extended -**01 */
M6809_INLINE void com_ex( void )
{
	UINT8 t;
	EXTBYTE(t); t = ~t;
	CLR_NZV; SET_NZ8(t); SEC;
	WM(EAD,t);
}

/* $74 LSR extended -0*-* */
M6809_INLINE void lsr_ex( void )
{
	UINT8 t;
	EXTBYTE(t); CLR_NZC; CC |= (t & CC_C);
	t>>=1; SET_Z8(t);
	WM(EAD,t);
}

/* $75 ILLEGAL */

/* $76 ROR extended -**-* */
M6809_INLINE void ror_ex( void )
{
	UINT8 t,r;
	EXTBYTE(t); r=(CC & CC_C) << 7;
	CLR_NZC; CC |= (t & CC_C);
	r |= t>>1; SET_NZ8(r);
	WM(EAD,r);
}

/* $77 ASR extended ?**-* */
M6809_INLINE void asr_ex( void )
{
	UINT8 t;
	EXTBYTE(t); CLR_NZC; CC |= (t & CC_C);
	t=(t&0x80)|(t>>1);
	SET_NZ8(t);
	WM(EAD,t);
}

/* $78 ASL extended ?**** */
M6809_INLINE void asl_ex( void )
{
	UINT16 t,r;
	EXTBYTE(t); r=t<<1;
	CLR_NZVC; SET_FLAGS8(t,t,r);
	WM(EAD,r);
}

/* $79 ROL extended -**** */
M6809_INLINE void rol_ex( void )
{
	UINT16 t,r;
	EXTBYTE(t); r = (CC & CC_C) | (t << 1);
	CLR_NZVC; SET_FLAGS8(t,t,r);
	WM(EAD,r);
}

/* $7A DEC extended -***- */
M6809_INLINE void dec_ex( void )
{
	UINT8 t;
	EXTBYTE(t); --t;
	CLR_NZV; SET_FLAGS8D(t);
	WM(EAD,t);
}

/* $7B ILLEGAL */

/* $7C INC extended -***- */
M6809_INLINE void inc_ex( void )
{
	UINT8 t;
	EXTBYTE(t); ++t;
	CLR_NZV; SET_FLAGS8I(t);
	WM(EAD,t);
}

/* $7D TST extended -**0- */
M6809_INLINE void tst_ex( void )
{
	UINT8 t;
	EXTBYTE(t); CLR_NZV; SET_NZ8(t);
}

/* $7E JMP extended ----- */
M6809_INLINE void jmp_ex( void )
{
	EXTENDED;
	PCD = EAD;
	CHANGE_PC;
}

/* $7F CLR extended -0100 */
M6809_INLINE void clr_ex( void )
{
	EXTENDED;
	(void)RM(EAD);
	WM(EAD,0);
	CLR_NZVC; SEZ;
}

/* $80 SUBA immediate ?**** */
M6809_INLINE void suba_im( void )
{
	UINT16 t,r;
	IMMBYTE(t);
	r = A - t;
	CLR_NZVC;
	SET_FLAGS8(A,t,r);
	A = r;
}

/* $81 CMPA immediate ?**** */
M6809_INLINE void cmpa_im( void )
{
	UINT16	  t,r;
	IMMBYTE(t);
	r = A - t;
	CLR_NZVC;
	SET_FLAGS8(A,t,r);
}

/* $82 SBCA immediate ?**** */
M6809_INLINE void sbca_im( void )
{
	UINT16	  t,r;
	IMMBYTE(t);
	r = A - t - (CC & CC_C);
	CLR_NZVC;
	SET_FLAGS8(A,t,r);
	A = r;
}

/* $83 SUBD (CMPD CMPU) immediate -**** */
M6809_INLINE void subd_im( void )
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

/* $1083 CMPD immediate -**** */
M6809_INLINE void cmpd_im( void )
{
	UINT32 r,d;
	PAIR b;
	IMMWORD(b);
	d = D;
	r = d - b.d;
	CLR_NZVC;
	SET_FLAGS16(d,b.d,r);
}

/* $1183 CMPU immediate -**** */
M6809_INLINE void cmpu_im( void )
{
	UINT32 r, d;
	PAIR b;
	IMMWORD(b);
	d = U;
	r = d - b.d;
	CLR_NZVC;
	SET_FLAGS16(d,b.d,r);
}

/* $84 ANDA immediate -**0- */
M6809_INLINE void anda_im( void )
{
	UINT8 t;
	IMMBYTE(t);
	A &= t;
	CLR_NZV;
	SET_NZ8(A);
}

/* $85 BITA immediate -**0- */
M6809_INLINE void bita_im( void )
{
	UINT8 t,r;
	IMMBYTE(t);
	r = A & t;
	CLR_NZV;
	SET_NZ8(r);
}

/* $86 LDA immediate -**0- */
M6809_INLINE void lda_im( void )
{
	IMMBYTE(A);
	CLR_NZV;
	SET_NZ8(A);
}

/* is this a legal instruction? */
/* $87 STA immediate -**0- */
M6809_INLINE void sta_im( void )
{
	CLR_NZV;
	SET_NZ8(A);
	IMM8;
	WM(EAD,A);
}

/* $88 EORA immediate -**0- */
M6809_INLINE void eora_im( void )
{
	UINT8 t;
	IMMBYTE(t);
	A ^= t;
	CLR_NZV;
	SET_NZ8(A);
}

/* $89 ADCA immediate ***** */
M6809_INLINE void adca_im( void )
{
	UINT16 t,r;
	IMMBYTE(t);
	r = A + t + (CC & CC_C);
	CLR_HNZVC;
	SET_FLAGS8(A,t,r);
	SET_H(A,t,r);
	A = r;
}

/* $8A ORA immediate -**0- */
M6809_INLINE void ora_im( void )
{
	UINT8 t;
	IMMBYTE(t);
	A |= t;
	CLR_NZV;
	SET_NZ8(A);
}

/* $8B ADDA immediate ***** */
M6809_INLINE void adda_im( void )
{
	UINT16 t,r;
	IMMBYTE(t);
	r = A + t;
	CLR_HNZVC;
	SET_FLAGS8(A,t,r);
	SET_H(A,t,r);
	A = r;
}

/* $8C CMPX (CMPY CMPS) immediate -**** */
M6809_INLINE void cmpx_im( void )
{
	UINT32 r,d;
	PAIR b;
	IMMWORD(b);
	d = X;
	r = d - b.d;
	CLR_NZVC;
	SET_FLAGS16(d,b.d,r);
}

/* $108C CMPY immediate -**** */
M6809_INLINE void cmpy_im( void )
{
	UINT32 r,d;
	PAIR b;
	IMMWORD(b);
	d = Y;
	r = d - b.d;
	CLR_NZVC;
	SET_FLAGS16(d,b.d,r);
}

/* $118C CMPS immediate -**** */
M6809_INLINE void cmps_im( void )
{
	UINT32 r,d;
	PAIR b;
	IMMWORD(b);
	d = S;
	r = d - b.d;
	CLR_NZVC;
	SET_FLAGS16(d,b.d,r);
}

/* $8D BSR ----- */
M6809_INLINE void bsr( void )
{
	UINT8 t;
	IMMBYTE(t);
	PUSHWORD(pPC);
	PC += SIGNED(t);
	CHANGE_PC;
}

/* $8E LDX (LDY) immediate -**0- */
M6809_INLINE void ldx_im( void )
{
	IMMWORD(pX);
	CLR_NZV;
	SET_NZ16(X);
}

/* $108E LDY immediate -**0- */
M6809_INLINE void ldy_im( void )
{
	IMMWORD(pY);
	CLR_NZV;
	SET_NZ16(Y);
}

/* is this a legal instruction? */
/* $8F STX (STY) immediate -**0- */
M6809_INLINE void stx_im( void )
{
	CLR_NZV;
	SET_NZ16(X);
	IMM16;
	WM16(EAD,&pX);
}

/* is this a legal instruction? */
/* $108F STY immediate -**0- */
M6809_INLINE void sty_im( void )
{
	CLR_NZV;
	SET_NZ16(Y);
	IMM16;
	WM16(EAD,&pY);
}

/* $90 SUBA direct ?**** */
M6809_INLINE void suba_di( void )
{
	UINT16	  t,r;
	DIRBYTE(t);
	r = A - t;
	CLR_NZVC;
	SET_FLAGS8(A,t,r);
	A = r;
}

/* $91 CMPA direct ?**** */
M6809_INLINE void cmpa_di( void )
{
	UINT16	  t,r;
	DIRBYTE(t);
	r = A - t;
	CLR_NZVC;
	SET_FLAGS8(A,t,r);
}

/* $92 SBCA direct ?**** */
M6809_INLINE void sbca_di( void )
{
	UINT16	  t,r;
	DIRBYTE(t);
	r = A - t - (CC & CC_C);
	CLR_NZVC;
	SET_FLAGS8(A,t,r);
	A = r;
}

/* $93 SUBD (CMPD CMPU) direct -**** */
M6809_INLINE void subd_di( void )
{
	UINT32 r,d;
	PAIR b;
	DIRWORD(b);
	d = D;
	r = d - b.d;
	CLR_NZVC;
	SET_FLAGS16(d,b.d,r);
	D = r;
}

/* $1093 CMPD direct -**** */
M6809_INLINE void cmpd_di( void )
{
	UINT32 r,d;
	PAIR b;
	DIRWORD(b);
	d = D;
	r = d - b.d;
	CLR_NZVC;
	SET_FLAGS16(d,b.d,r);
}

/* $1193 CMPU direct -**** */
M6809_INLINE void cmpu_di( void )
{
	UINT32 r,d;
	PAIR b;
	DIRWORD(b);
	d = U;
	r = d - b.d;
	CLR_NZVC;
	SET_FLAGS16(U,b.d,r);
}

/* $94 ANDA direct -**0- */
M6809_INLINE void anda_di( void )
{
	UINT8 t;
	DIRBYTE(t);
	A &= t;
	CLR_NZV;
	SET_NZ8(A);
}

/* $95 BITA direct -**0- */
M6809_INLINE void bita_di( void )
{
	UINT8 t,r;
	DIRBYTE(t);
	r = A & t;
	CLR_NZV;
	SET_NZ8(r);
}

/* $96 LDA direct -**0- */
M6809_INLINE void lda_di( void )
{
	DIRBYTE(A);
	CLR_NZV;
	SET_NZ8(A);
}

/* $97 STA direct -**0- */
M6809_INLINE void sta_di( void )
{
	CLR_NZV;
	SET_NZ8(A);
	DIRECT;
	WM(EAD,A);
}

/* $98 EORA direct -**0- */
M6809_INLINE void eora_di( void )
{
	UINT8 t;
	DIRBYTE(t);
	A ^= t;
	CLR_NZV;
	SET_NZ8(A);
}

/* $99 ADCA direct ***** */
M6809_INLINE void adca_di( void )
{
	UINT16 t,r;
	DIRBYTE(t);
	r = A + t + (CC & CC_C);
	CLR_HNZVC;
	SET_FLAGS8(A,t,r);
	SET_H(A,t,r);
	A = r;
}

/* $9A ORA direct -**0- */
M6809_INLINE void ora_di( void )
{
	UINT8 t;
	DIRBYTE(t);
	A |= t;
	CLR_NZV;
	SET_NZ8(A);
}

/* $9B ADDA direct ***** */
M6809_INLINE void adda_di( void )
{
	UINT16 t,r;
	DIRBYTE(t);
	r = A + t;
	CLR_HNZVC;
	SET_FLAGS8(A,t,r);
	SET_H(A,t,r);
	A = r;
}

/* $9C CMPX (CMPY CMPS) direct -**** */
M6809_INLINE void cmpx_di( void )
{
	UINT32 r,d;
	PAIR b;
	DIRWORD(b);
	d = X;
	r = d - b.d;
	CLR_NZVC;
	SET_FLAGS16(d,b.d,r);
}

/* $109C CMPY direct -**** */
M6809_INLINE void cmpy_di( void )
{
	UINT32 r,d;
	PAIR b;
	DIRWORD(b);
	d = Y;
	r = d - b.d;
	CLR_NZVC;
	SET_FLAGS16(d,b.d,r);
}

/* $119C CMPS direct -**** */
M6809_INLINE void cmps_di( void )
{
	UINT32 r,d;
	PAIR b;
	DIRWORD(b);
	d = S;
	r = d - b.d;
	CLR_NZVC;
	SET_FLAGS16(d,b.d,r);
}

/* $9D JSR direct ----- */
M6809_INLINE void jsr_di( void )
{
	DIRECT;
	PUSHWORD(pPC);
	PCD = EAD;
	CHANGE_PC;
}

/* $9E LDX (LDY) direct -**0- */
M6809_INLINE void ldx_di( void )
{
	DIRWORD(pX);
	CLR_NZV;
	SET_NZ16(X);
}

/* $109E LDY direct -**0- */
M6809_INLINE void ldy_di( void )
{
	DIRWORD(pY);
	CLR_NZV;
	SET_NZ16(Y);
}

/* $9F STX (STY) direct -**0- */
M6809_INLINE void stx_di( void )
{
	CLR_NZV;
	SET_NZ16(X);
	DIRECT;
	WM16(EAD,&pX);
}

/* $109F STY direct -**0- */
M6809_INLINE void sty_di( void )
{
	CLR_NZV;
	SET_NZ16(Y);
	DIRECT;
	WM16(EAD,&pY);
}

/* $a0 SUBA indexed ?**** */
M6809_INLINE void suba_ix( void )
{
	UINT16 t,r;
	fetch_effective_address();
	t = RM(EAD);
	r = A - t;
	CLR_NZVC;
	SET_FLAGS8(A,t,r);
	A = r;
}

/* $a1 CMPA indexed ?**** */
M6809_INLINE void cmpa_ix( void )
{
	UINT16 t,r;
	fetch_effective_address();
	t = RM(EAD);
	r = A - t;
	CLR_NZVC;
	SET_FLAGS8(A,t,r);
}

/* $a2 SBCA indexed ?**** */
M6809_INLINE void sbca_ix( void )
{
	UINT16	  t,r;
	fetch_effective_address();
	t = RM(EAD);
	r = A - t - (CC & CC_C);
	CLR_NZVC;
	SET_FLAGS8(A,t,r);
	A = r;
}

/* $a3 SUBD (CMPD CMPU) indexed -**** */
M6809_INLINE void subd_ix( void )
{
	UINT32 r,d;
	PAIR b;
	fetch_effective_address();
    b.d=RM16(EAD);
	d = D;
	r = d - b.d;
	CLR_NZVC;
	SET_FLAGS16(d,b.d,r);
	D = r;
}

/* $10a3 CMPD indexed -**** */
M6809_INLINE void cmpd_ix( void )
{
	UINT32 r,d;
	PAIR b;
	fetch_effective_address();
    b.d=RM16(EAD);
	d = D;
	r = d - b.d;
	CLR_NZVC;
	SET_FLAGS16(d,b.d,r);
}

/* $11a3 CMPU indexed -**** */
M6809_INLINE void cmpu_ix( void )
{
	UINT32 r;
	PAIR b;
	fetch_effective_address();
    b.d=RM16(EAD);
	r = U - b.d;
	CLR_NZVC;
	SET_FLAGS16(U,b.d,r);
}

/* $a4 ANDA indexed -**0- */
M6809_INLINE void anda_ix( void )
{
	fetch_effective_address();
	A &= RM(EAD);
	CLR_NZV;
	SET_NZ8(A);
}

/* $a5 BITA indexed -**0- */
M6809_INLINE void bita_ix( void )
{
	UINT8 r;
	fetch_effective_address();
	r = A & RM(EAD);
	CLR_NZV;
	SET_NZ8(r);
}

/* $a6 LDA indexed -**0- */
M6809_INLINE void lda_ix( void )
{
	fetch_effective_address();
	A = RM(EAD);
	CLR_NZV;
	SET_NZ8(A);
}

/* $a7 STA indexed -**0- */
M6809_INLINE void sta_ix( void )
{
	fetch_effective_address();
    CLR_NZV;
	SET_NZ8(A);
	WM(EAD,A);
}

/* $a8 EORA indexed -**0- */
M6809_INLINE void eora_ix( void )
{
	fetch_effective_address();
	A ^= RM(EAD);
	CLR_NZV;
	SET_NZ8(A);
}

/* $a9 ADCA indexed ***** */
M6809_INLINE void adca_ix( void )
{
	UINT16 t,r;
	fetch_effective_address();
	t = RM(EAD);
	r = A + t + (CC & CC_C);
	CLR_HNZVC;
	SET_FLAGS8(A,t,r);
	SET_H(A,t,r);
	A = r;
}

/* $aA ORA indexed -**0- */
M6809_INLINE void ora_ix( void )
{
	fetch_effective_address();
	A |= RM(EAD);
	CLR_NZV;
	SET_NZ8(A);
}

/* $aB ADDA indexed ***** */
M6809_INLINE void adda_ix( void )
{
	UINT16 t,r;
	fetch_effective_address();
	t = RM(EAD);
	r = A + t;
	CLR_HNZVC;
	SET_FLAGS8(A,t,r);
	SET_H(A,t,r);
	A = r;
}

/* $aC CMPX (CMPY CMPS) indexed -**** */
M6809_INLINE void cmpx_ix( void )
{
	UINT32 r,d;
	PAIR b;
	fetch_effective_address();
    b.d=RM16(EAD);
	d = X;
	r = d - b.d;
	CLR_NZVC;
	SET_FLAGS16(d,b.d,r);
}

/* $10aC CMPY indexed -**** */
M6809_INLINE void cmpy_ix( void )
{
	UINT32 r,d;
	PAIR b;
	fetch_effective_address();
    b.d=RM16(EAD);
	d = Y;
	r = d - b.d;
	CLR_NZVC;
	SET_FLAGS16(d,b.d,r);
}

/* $11aC CMPS indexed -**** */
M6809_INLINE void cmps_ix( void )
{
	UINT32 r,d;
	PAIR b;
	fetch_effective_address();
    b.d=RM16(EAD);
	d = S;
	r = d - b.d;
	CLR_NZVC;
	SET_FLAGS16(d,b.d,r);
}

/* $aD JSR indexed ----- */
M6809_INLINE void jsr_ix( void )
{
	fetch_effective_address();
    PUSHWORD(pPC);
	PCD = EAD;
	CHANGE_PC;
}

/* $aE LDX (LDY) indexed -**0- */
M6809_INLINE void ldx_ix( void )
{
	fetch_effective_address();
    X=RM16(EAD);
	CLR_NZV;
	SET_NZ16(X);
}

/* $10aE LDY indexed -**0- */
M6809_INLINE void ldy_ix( void )
{
	fetch_effective_address();
    Y=RM16(EAD);
	CLR_NZV;
	SET_NZ16(Y);
}

/* $aF STX (STY) indexed -**0- */
M6809_INLINE void stx_ix( void )
{
	fetch_effective_address();
    CLR_NZV;
	SET_NZ16(X);
	WM16(EAD,&pX);
}

/* $10aF STY indexed -**0- */
M6809_INLINE void sty_ix( void )
{
	fetch_effective_address();
    CLR_NZV;
	SET_NZ16(Y);
	WM16(EAD,&pY);
}

/* $b0 SUBA extended ?**** */
M6809_INLINE void suba_ex( void )
{
	UINT16	  t,r;
	EXTBYTE(t);
	r = A - t;
	CLR_NZVC;
	SET_FLAGS8(A,t,r);
	A = r;
}

/* $b1 CMPA extended ?**** */
M6809_INLINE void cmpa_ex( void )
{
	UINT16	  t,r;
	EXTBYTE(t);
	r = A - t;
	CLR_NZVC;
	SET_FLAGS8(A,t,r);
}

/* $b2 SBCA extended ?**** */
M6809_INLINE void sbca_ex( void )
{
	UINT16	  t,r;
	EXTBYTE(t);
	r = A - t - (CC & CC_C);
	CLR_NZVC;
	SET_FLAGS8(A,t,r);
	A = r;
}

/* $b3 SUBD (CMPD CMPU) extended -**** */
M6809_INLINE void subd_ex( void )
{
	UINT32 r,d;
	PAIR b;
	EXTWORD(b);
	d = D;
	r = d - b.d;
	CLR_NZVC;
	SET_FLAGS16(d,b.d,r);
	D = r;
}

/* $10b3 CMPD extended -**** */
M6809_INLINE void cmpd_ex( void )
{
	UINT32 r,d;
	PAIR b;
	EXTWORD(b);
	d = D;
	r = d - b.d;
	CLR_NZVC;
	SET_FLAGS16(d,b.d,r);
}

/* $11b3 CMPU extended -**** */
M6809_INLINE void cmpu_ex( void )
{
	UINT32 r,d;
	PAIR b;
	EXTWORD(b);
	d = U;
	r = d - b.d;
	CLR_NZVC;
	SET_FLAGS16(d,b.d,r);
}

/* $b4 ANDA extended -**0- */
M6809_INLINE void anda_ex( void )
{
	UINT8 t;
	EXTBYTE(t);
	A &= t;
	CLR_NZV;
	SET_NZ8(A);
}

/* $b5 BITA extended -**0- */
M6809_INLINE void bita_ex( void )
{
	UINT8 t,r;
	EXTBYTE(t);
	r = A & t;
	CLR_NZV; SET_NZ8(r);
}

/* $b6 LDA extended -**0- */
M6809_INLINE void lda_ex( void )
{
	EXTBYTE(A);
	CLR_NZV;
	SET_NZ8(A);
}

/* $b7 STA extended -**0- */
M6809_INLINE void sta_ex( void )
{
	CLR_NZV;
	SET_NZ8(A);
	EXTENDED;
	WM(EAD,A);
}

/* $b8 EORA extended -**0- */
M6809_INLINE void eora_ex( void )
{
	UINT8 t;
	EXTBYTE(t);
	A ^= t;
	CLR_NZV;
	SET_NZ8(A);
}

/* $b9 ADCA extended ***** */
M6809_INLINE void adca_ex( void )
{
	UINT16 t,r;
	EXTBYTE(t);
	r = A + t + (CC & CC_C);
	CLR_HNZVC;
	SET_FLAGS8(A,t,r);
	SET_H(A,t,r);
	A = r;
}

/* $bA ORA extended -**0- */
M6809_INLINE void ora_ex( void )
{
	UINT8 t;
	EXTBYTE(t);
	A |= t;
	CLR_NZV;
	SET_NZ8(A);
}

/* $bB ADDA extended ***** */
M6809_INLINE void adda_ex( void )
{
	UINT16 t,r;
	EXTBYTE(t);
	r = A + t;
	CLR_HNZVC;
	SET_FLAGS8(A,t,r);
	SET_H(A,t,r);
	A = r;
}

/* $bC CMPX (CMPY CMPS) extended -**** */
M6809_INLINE void cmpx_ex( void )
{
	UINT32 r,d;
	PAIR b;
	EXTWORD(b);
	d = X;
	r = d - b.d;
	CLR_NZVC;
	SET_FLAGS16(d,b.d,r);
}

/* $10bC CMPY extended -**** */
M6809_INLINE void cmpy_ex( void )
{
	UINT32 r,d;
	PAIR b;
	EXTWORD(b);
	d = Y;
	r = d - b.d;
	CLR_NZVC;
	SET_FLAGS16(d,b.d,r);
}

/* $11bC CMPS extended -**** */
M6809_INLINE void cmps_ex( void )
{
	UINT32 r,d;
	PAIR b;
	EXTWORD(b);
	d = S;
	r = d - b.d;
	CLR_NZVC;
	SET_FLAGS16(d,b.d,r);
}

/* $bD JSR extended ----- */
M6809_INLINE void jsr_ex( void )
{
	EXTENDED;
	PUSHWORD(pPC);
	PCD = EAD;
	CHANGE_PC;
}

/* $bE LDX (LDY) extended -**0- */
M6809_INLINE void ldx_ex( void )
{
	EXTWORD(pX);
	CLR_NZV;
	SET_NZ16(X);
}

/* $10bE LDY extended -**0- */
M6809_INLINE void ldy_ex( void )
{
	EXTWORD(pY);
	CLR_NZV;
	SET_NZ16(Y);
}

/* $bF STX (STY) extended -**0- */
M6809_INLINE void stx_ex( void )
{
	CLR_NZV;
	SET_NZ16(X);
	EXTENDED;
	WM16(EAD,&pX);
}

/* $10bF STY extended -**0- */
M6809_INLINE void sty_ex( void )
{
	CLR_NZV;
	SET_NZ16(Y);
	EXTENDED;
	WM16(EAD,&pY);
}

/* $c0 SUBB immediate ?**** */
M6809_INLINE void subb_im( void )
{
	UINT16	  t,r;
	IMMBYTE(t);
	r = B - t;
	CLR_NZVC;
	SET_FLAGS8(B,t,r);
	B = r;
}

/* $c1 CMPB immediate ?**** */
M6809_INLINE void cmpb_im( void )
{
	UINT16	  t,r;
	IMMBYTE(t);
	r = B - t;
	CLR_NZVC; SET_FLAGS8(B,t,r);
}

/* $c2 SBCB immediate ?**** */
M6809_INLINE void sbcb_im( void )
{
	UINT16	  t,r;
	IMMBYTE(t);
	r = B - t - (CC & CC_C);
	CLR_NZVC;
	SET_FLAGS8(B,t,r);
	B = r;
}

/* $c3 ADDD immediate -**** */
M6809_INLINE void addd_im( void )
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
M6809_INLINE void andb_im( void )
{
	UINT8 t;
	IMMBYTE(t);
	B &= t;
	CLR_NZV;
	SET_NZ8(B);
}

/* $c5 BITB immediate -**0- */
M6809_INLINE void bitb_im( void )
{
	UINT8 t,r;
	IMMBYTE(t);
	r = B & t;
	CLR_NZV;
	SET_NZ8(r);
}

/* $c6 LDB immediate -**0- */
M6809_INLINE void ldb_im( void )
{
	IMMBYTE(B);
	CLR_NZV;
	SET_NZ8(B);
}

/* is this a legal instruction? */
/* $c7 STB immediate -**0- */
M6809_INLINE void stb_im( void )
{
	CLR_NZV;
	SET_NZ8(B);
	IMM8;
	WM(EAD,B);
}

/* $c8 EORB immediate -**0- */
M6809_INLINE void eorb_im( void )
{
	UINT8 t;
	IMMBYTE(t);
	B ^= t;
	CLR_NZV;
	SET_NZ8(B);
}

/* $c9 ADCB immediate ***** */
M6809_INLINE void adcb_im( void )
{
	UINT16 t,r;
	IMMBYTE(t);
	r = B + t + (CC & CC_C);
	CLR_HNZVC;
	SET_FLAGS8(B,t,r);
	SET_H(B,t,r);
	B = r;
}

/* $cA ORB immediate -**0- */
M6809_INLINE void orb_im( void )
{
	UINT8 t;
	IMMBYTE(t);
	B |= t;
	CLR_NZV;
	SET_NZ8(B);
}

/* $cB ADDB immediate ***** */
M6809_INLINE void addb_im( void )
{
	UINT16 t,r;
	IMMBYTE(t);
	r = B + t;
	CLR_HNZVC;
	SET_FLAGS8(B,t,r);
	SET_H(B,t,r);
	B = r;
}

/* $cC LDD immediate -**0- */
M6809_INLINE void ldd_im( void )
{
	IMMWORD(pD);
	CLR_NZV;
	SET_NZ16(D);
}

/* is this a legal instruction? */
/* $cD STD immediate -**0- */
M6809_INLINE void std_im( void )
{
	CLR_NZV;
	SET_NZ16(D);
    IMM16;
	WM16(EAD,&pD);
}

/* $cE LDU (LDS) immediate -**0- */
M6809_INLINE void ldu_im( void )
{
	IMMWORD(pU);
	CLR_NZV;
	SET_NZ16(U);
}

/* $10cE LDS immediate -**0- */
M6809_INLINE void lds_im( void )
{
	IMMWORD(pS);
	CLR_NZV;
	SET_NZ16(S);
	m6809.int_state |= M6809_LDS;
}

/* is this a legal instruction? */
/* $cF STU (STS) immediate -**0- */
M6809_INLINE void stu_im( void )
{
	CLR_NZV;
	SET_NZ16(U);
    IMM16;
	WM16(EAD,&pU);
}

/* is this a legal instruction? */
/* $10cF STS immediate -**0- */
M6809_INLINE void sts_im( void )
{
	CLR_NZV;
	SET_NZ16(S);
    IMM16;
	WM16(EAD,&pS);
}

/* $d0 SUBB direct ?**** */
M6809_INLINE void subb_di( void )
{
	UINT16	  t,r;
	DIRBYTE(t);
	r = B - t;
	CLR_NZVC;
	SET_FLAGS8(B,t,r);
	B = r;
}

/* $d1 CMPB direct ?**** */
M6809_INLINE void cmpb_di( void )
{
	UINT16	  t,r;
	DIRBYTE(t);
	r = B - t;
	CLR_NZVC;
	SET_FLAGS8(B,t,r);
}

/* $d2 SBCB direct ?**** */
M6809_INLINE void sbcb_di( void )
{
	UINT16	  t,r;
	DIRBYTE(t);
	r = B - t - (CC & CC_C);
	CLR_NZVC;
	SET_FLAGS8(B,t,r);
	B = r;
}

/* $d3 ADDD direct -**** */
M6809_INLINE void addd_di( void )
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
M6809_INLINE void andb_di( void )
{
	UINT8 t;
	DIRBYTE(t);
	B &= t;
	CLR_NZV;
	SET_NZ8(B);
}

/* $d5 BITB direct -**0- */
M6809_INLINE void bitb_di( void )
{
	UINT8 t,r;
	DIRBYTE(t);
	r = B & t;
	CLR_NZV;
	SET_NZ8(r);
}

/* $d6 LDB direct -**0- */
M6809_INLINE void ldb_di( void )
{
	DIRBYTE(B);
	CLR_NZV;
	SET_NZ8(B);
}

/* $d7 STB direct -**0- */
M6809_INLINE void stb_di( void )
{
	CLR_NZV;
	SET_NZ8(B);
	DIRECT;
	WM(EAD,B);
}

/* $d8 EORB direct -**0- */
M6809_INLINE void eorb_di( void )
{
	UINT8 t;
	DIRBYTE(t);
	B ^= t;
	CLR_NZV;
	SET_NZ8(B);
}

/* $d9 ADCB direct ***** */
M6809_INLINE void adcb_di( void )
{
	UINT16 t,r;
	DIRBYTE(t);
	r = B + t + (CC & CC_C);
	CLR_HNZVC;
	SET_FLAGS8(B,t,r);
	SET_H(B,t,r);
	B = r;
}

/* $dA ORB direct -**0- */
M6809_INLINE void orb_di( void )
{
	UINT8 t;
	DIRBYTE(t);
	B |= t;
	CLR_NZV;
	SET_NZ8(B);
}

/* $dB ADDB direct ***** */
M6809_INLINE void addb_di( void )
{
	UINT16 t,r;
	DIRBYTE(t);
	r = B + t;
	CLR_HNZVC;
	SET_FLAGS8(B,t,r);
	SET_H(B,t,r);
	B = r;
}

/* $dC LDD direct -**0- */
M6809_INLINE void ldd_di( void )
{
	DIRWORD(pD);
	CLR_NZV;
	SET_NZ16(D);
}

/* $dD STD direct -**0- */
M6809_INLINE void std_di( void )
{
	CLR_NZV;
	SET_NZ16(D);
    DIRECT;
	WM16(EAD,&pD);
}

/* $dE LDU (LDS) direct -**0- */
M6809_INLINE void ldu_di( void )
{
	DIRWORD(pU);
	CLR_NZV;
	SET_NZ16(U);
}

/* $10dE LDS direct -**0- */
M6809_INLINE void lds_di( void )
{
	DIRWORD(pS);
	CLR_NZV;
	SET_NZ16(S);
	m6809.int_state |= M6809_LDS;
}

/* $dF STU (STS) direct -**0- */
M6809_INLINE void stu_di( void )
{
	CLR_NZV;
	SET_NZ16(U);
	DIRECT;
	WM16(EAD,&pU);
}

/* $10dF STS direct -**0- */
M6809_INLINE void sts_di( void )
{
	CLR_NZV;
	SET_NZ16(S);
	DIRECT;
	WM16(EAD,&pS);
}

/* $e0 SUBB indexed ?**** */
M6809_INLINE void subb_ix( void )
{
	UINT16	  t,r;
	fetch_effective_address();
	t = RM(EAD);
	r = B - t;
	CLR_NZVC;
	SET_FLAGS8(B,t,r);
	B = r;
}

/* $e1 CMPB indexed ?**** */
M6809_INLINE void cmpb_ix( void )
{
	UINT16	  t,r;
	fetch_effective_address();
	t = RM(EAD);
	r = B - t;
	CLR_NZVC;
	SET_FLAGS8(B,t,r);
}

/* $e2 SBCB indexed ?**** */
M6809_INLINE void sbcb_ix( void )
{
	UINT16	  t,r;
	fetch_effective_address();
	t = RM(EAD);
	r = B - t - (CC & CC_C);
	CLR_NZVC;
	SET_FLAGS8(B,t,r);
	B = r;
}

/* $e3 ADDD indexed -**** */
M6809_INLINE void addd_ix( void )
{
	UINT32 r,d;
    PAIR b;
    fetch_effective_address();
	b.d=RM16(EAD);
	d = D;
	r = d + b.d;
	CLR_NZVC;
	SET_FLAGS16(d,b.d,r);
	D = r;
}

/* $e4 ANDB indexed -**0- */
M6809_INLINE void andb_ix( void )
{
	fetch_effective_address();
	B &= RM(EAD);
	CLR_NZV;
	SET_NZ8(B);
}

/* $e5 BITB indexed -**0- */
M6809_INLINE void bitb_ix( void )
{
	UINT8 r;
	fetch_effective_address();
	r = B & RM(EAD);
	CLR_NZV;
	SET_NZ8(r);
}

/* $e6 LDB indexed -**0- */
M6809_INLINE void ldb_ix( void )
{
	fetch_effective_address();
	B = RM(EAD);
	CLR_NZV;
	SET_NZ8(B);
}

/* $e7 STB indexed -**0- */
M6809_INLINE void stb_ix( void )
{
	fetch_effective_address();
    CLR_NZV;
	SET_NZ8(B);
	WM(EAD,B);
}

/* $e8 EORB indexed -**0- */
M6809_INLINE void eorb_ix( void )
{
	fetch_effective_address();
	B ^= RM(EAD);
	CLR_NZV;
	SET_NZ8(B);
}

/* $e9 ADCB indexed ***** */
M6809_INLINE void adcb_ix( void )
{
	UINT16 t,r;
	fetch_effective_address();
	t = RM(EAD);
	r = B + t + (CC & CC_C);
	CLR_HNZVC;
	SET_FLAGS8(B,t,r);
	SET_H(B,t,r);
	B = r;
}

/* $eA ORB indexed -**0- */
M6809_INLINE void orb_ix( void )
{
	fetch_effective_address();
	B |= RM(EAD);
	CLR_NZV;
	SET_NZ8(B);
}

/* $eB ADDB indexed ***** */
M6809_INLINE void addb_ix( void )
{
	UINT16 t,r;
	fetch_effective_address();
	t = RM(EAD);
	r = B + t;
	CLR_HNZVC;
	SET_FLAGS8(B,t,r);
	SET_H(B,t,r);
	B = r;
}

/* $eC LDD indexed -**0- */
M6809_INLINE void ldd_ix( void )
{
	fetch_effective_address();
    D=RM16(EAD);
	CLR_NZV; SET_NZ16(D);
}

/* $eD STD indexed -**0- */
M6809_INLINE void std_ix( void )
{
	fetch_effective_address();
    CLR_NZV;
	SET_NZ16(D);
	WM16(EAD,&pD);
}

/* $eE LDU (LDS) indexed -**0- */
M6809_INLINE void ldu_ix( void )
{
	fetch_effective_address();
    U=RM16(EAD);
	CLR_NZV;
	SET_NZ16(U);
}

/* $10eE LDS indexed -**0- */
M6809_INLINE void lds_ix( void )
{
	fetch_effective_address();
    S=RM16(EAD);
	CLR_NZV;
	SET_NZ16(S);
	m6809.int_state |= M6809_LDS;
}

/* $eF STU (STS) indexed -**0- */
M6809_INLINE void stu_ix( void )
{
	fetch_effective_address();
    CLR_NZV;
	SET_NZ16(U);
	WM16(EAD,&pU);
}

/* $10eF STS indexed -**0- */
M6809_INLINE void sts_ix( void )
{
	fetch_effective_address();
    CLR_NZV;
	SET_NZ16(S);
	WM16(EAD,&pS);
}

/* $f0 SUBB extended ?**** */
M6809_INLINE void subb_ex( void )
{
	UINT16	  t,r;
	EXTBYTE(t);
	r = B - t;
	CLR_NZVC;
	SET_FLAGS8(B,t,r);
	B = r;
}

/* $f1 CMPB extended ?**** */
M6809_INLINE void cmpb_ex( void )
{
	UINT16	  t,r;
	EXTBYTE(t);
	r = B - t;
	CLR_NZVC;
	SET_FLAGS8(B,t,r);
}

/* $f2 SBCB extended ?**** */
M6809_INLINE void sbcb_ex( void )
{
	UINT16	  t,r;
	EXTBYTE(t);
	r = B - t - (CC & CC_C);
	CLR_NZVC;
	SET_FLAGS8(B,t,r);
	B = r;
}

/* $f3 ADDD extended -**** */
M6809_INLINE void addd_ex( void )
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
M6809_INLINE void andb_ex( void )
{
	UINT8 t;
	EXTBYTE(t);
	B &= t;
	CLR_NZV;
	SET_NZ8(B);
}

/* $f5 BITB extended -**0- */
M6809_INLINE void bitb_ex( void )
{
	UINT8 t,r;
	EXTBYTE(t);
	r = B & t;
	CLR_NZV;
	SET_NZ8(r);
}

/* $f6 LDB extended -**0- */
M6809_INLINE void ldb_ex( void )
{
	EXTBYTE(B);
	CLR_NZV;
	SET_NZ8(B);
}

/* $f7 STB extended -**0- */
M6809_INLINE void stb_ex( void )
{
	CLR_NZV;
	SET_NZ8(B);
	EXTENDED;
	WM(EAD,B);
}

/* $f8 EORB extended -**0- */
M6809_INLINE void eorb_ex( void )
{
	UINT8 t;
	EXTBYTE(t);
	B ^= t;
	CLR_NZV;
	SET_NZ8(B);
}

/* $f9 ADCB extended ***** */
M6809_INLINE void adcb_ex( void )
{
	UINT16 t,r;
	EXTBYTE(t);
	r = B + t + (CC & CC_C);
	CLR_HNZVC;
	SET_FLAGS8(B,t,r);
	SET_H(B,t,r);
	B = r;
}

/* $fA ORB extended -**0- */
M6809_INLINE void orb_ex( void )
{
	UINT8 t;
	EXTBYTE(t);
	B |= t;
	CLR_NZV;
	SET_NZ8(B);
}

/* $fB ADDB extended ***** */
M6809_INLINE void addb_ex( void )
{
	UINT16 t,r;
	EXTBYTE(t);
	r = B + t;
	CLR_HNZVC;
	SET_FLAGS8(B,t,r);
	SET_H(B,t,r);
	B = r;
}

/* $fC LDD extended -**0- */
M6809_INLINE void ldd_ex( void )
{
	EXTWORD(pD);
	CLR_NZV;
	SET_NZ16(D);
}

/* $fD STD extended -**0- */
M6809_INLINE void std_ex( void )
{
	CLR_NZV;
	SET_NZ16(D);
    EXTENDED;
	WM16(EAD,&pD);
}

/* $fE LDU (LDS) extended -**0- */
M6809_INLINE void ldu_ex( void )
{
	EXTWORD(pU);
	CLR_NZV;
	SET_NZ16(U);
}

/* $10fE LDS extended -**0- */
M6809_INLINE void lds_ex( void )
{
	EXTWORD(pS);
	CLR_NZV;
	SET_NZ16(S);
	m6809.int_state |= M6809_LDS;
}

/* $fF STU (STS) extended -**0- */
M6809_INLINE void stu_ex( void )
{
	CLR_NZV;
	SET_NZ16(U);
	EXTENDED;
	WM16(EAD,&pU);
}

/* $10fF STS extended -**0- */
M6809_INLINE void sts_ex( void )
{
	CLR_NZV;
	SET_NZ16(S);
	EXTENDED;
	WM16(EAD,&pS);
}

/* $10xx opcodes */
M6809_INLINE void pref10( void )
{
	UINT8 ireg2 = ROP(PCD);
	PC++;
	switch( ireg2 )
	{
		case 0x21: lbrn();		m6809_ICount-=5;	break;
		case 0x22: lbhi();		m6809_ICount-=5;	break;
		case 0x23: lbls();		m6809_ICount-=5;	break;
		case 0x24: lbcc();		m6809_ICount-=5;	break;
		case 0x25: lbcs();		m6809_ICount-=5;	break;
		case 0x26: lbne();		m6809_ICount-=5;	break;
		case 0x27: lbeq();		m6809_ICount-=5;	break;
		case 0x28: lbvc();		m6809_ICount-=5;	break;
		case 0x29: lbvs();		m6809_ICount-=5;	break;
		case 0x2a: lbpl();		m6809_ICount-=5;	break;
		case 0x2b: lbmi();		m6809_ICount-=5;	break;
		case 0x2c: lbge();		m6809_ICount-=5;	break;
		case 0x2d: lblt();		m6809_ICount-=5;	break;
		case 0x2e: lbgt();		m6809_ICount-=5;	break;
		case 0x2f: lble();		m6809_ICount-=5;	break;

		case 0x3f: swi2();		m6809_ICount-=20;	break;

		case 0x83: cmpd_im();	m6809_ICount-=5;	break;
		case 0x8c: cmpy_im();	m6809_ICount-=5;	break;
		case 0x8e: ldy_im();	m6809_ICount-=4;	break;
		case 0x8f: sty_im();	m6809_ICount-=4;	break;

		case 0x93: cmpd_di();	m6809_ICount-=7;	break;
		case 0x9c: cmpy_di();	m6809_ICount-=7;	break;
		case 0x9e: ldy_di();	m6809_ICount-=6;	break;
		case 0x9f: sty_di();	m6809_ICount-=6;	break;

		case 0xa3: cmpd_ix();	m6809_ICount-=7;	break;
		case 0xac: cmpy_ix();	m6809_ICount-=7;	break;
		case 0xae: ldy_ix();	m6809_ICount-=6;	break;
		case 0xaf: sty_ix();	m6809_ICount-=6;	break;

		case 0xb3: cmpd_ex();	m6809_ICount-=8;	break;
		case 0xbc: cmpy_ex();	m6809_ICount-=8;	break;
		case 0xbe: ldy_ex();	m6809_ICount-=7;	break;
		case 0xbf: sty_ex();	m6809_ICount-=7;	break;

		case 0xce: lds_im();	m6809_ICount-=4;	break;
		case 0xcf: sts_im();	m6809_ICount-=4;	break;

		case 0xde: lds_di();	m6809_ICount-=6;	break;
		case 0xdf: sts_di();	m6809_ICount-=6;	break;

		case 0xee: lds_ix();	m6809_ICount-=6;	break;
		case 0xef: sts_ix();	m6809_ICount-=6;	break;

		case 0xfe: lds_ex();	m6809_ICount-=7;	break;
		case 0xff: sts_ex();	m6809_ICount-=7;	break;

		default:   illegal();						break;
	}
}

/* $11xx opcodes */
M6809_INLINE void pref11( void )
{
	UINT8 ireg2 = ROP(PCD);
	PC++;
	switch( ireg2 )
	{
		case 0x3f: swi3();		m6809_ICount-=20;	break;

		case 0x83: cmpu_im();	m6809_ICount-=5;	break;
		case 0x8c: cmps_im();	m6809_ICount-=5;	break;

		case 0x93: cmpu_di();	m6809_ICount-=7;	break;
		case 0x9c: cmps_di();	m6809_ICount-=7;	break;

		case 0xa3: cmpu_ix();	m6809_ICount-=7;	break;
		case 0xac: cmps_ix();	m6809_ICount-=7;	break;

		case 0xb3: cmpu_ex();	m6809_ICount-=8;	break;
		case 0xbc: cmps_ex();	m6809_ICount-=8;	break;

		default:   illegal();						break;
	}
}




#ifdef __cplusplus
}
#endif

/* execute instructions on this CPU until icount expires */
int m6809_execute(int cycles)	/* NS 970908 */
{
    m6809_ICount = cycles - m6809.extra_cycles;
	m6809.extra_cycles = 0;

	if (m6809.int_state & (M6809_CWAI | M6809_SYNC))
	{
//		debugger_instruction_hook(Machine, PCD);
		m6809_ICount = 0;
	}
	else
	{
		do
		{
			pPPC = pPC;

//			debugger_instruction_hook(Machine, PCD);

			m6809.ireg = ROP(PCD);
			PC++;
#if BIG_SWITCH
            switch( m6809.ireg )
			{
			case 0x00: neg_di();   m6809_ICount-= 6; break;
			case 0x01: neg_di();   m6809_ICount-= 6; break;	/* undocumented */
			case 0x02: illegal();  m6809_ICount-= 2; break;
			case 0x03: com_di();   m6809_ICount-= 6; break;
			case 0x04: lsr_di();   m6809_ICount-= 6; break;
			case 0x05: illegal();  m6809_ICount-= 2; break;
			case 0x06: ror_di();   m6809_ICount-= 6; break;
			case 0x07: asr_di();   m6809_ICount-= 6; break;
			case 0x08: asl_di();   m6809_ICount-= 6; break;
			case 0x09: rol_di();   m6809_ICount-= 6; break;
			case 0x0a: dec_di();   m6809_ICount-= 6; break;
			case 0x0b: illegal();  m6809_ICount-= 2; break;
			case 0x0c: inc_di();   m6809_ICount-= 6; break;
			case 0x0d: tst_di();   m6809_ICount-= 6; break;
			case 0x0e: jmp_di();   m6809_ICount-= 3; break;
			case 0x0f: clr_di();   m6809_ICount-= 6; break;
			case 0x10: pref10();					 break;
			case 0x11: pref11();					 break;
			case 0x12: nop();	   m6809_ICount-= 2; break;
			case 0x13: sync();	   m6809_ICount-= 4; break;
			case 0x14: illegal();  m6809_ICount-= 2; break;
			case 0x15: illegal();  m6809_ICount-= 2; break;
			case 0x16: lbra();	   m6809_ICount-= 5; break;
			case 0x17: lbsr();	   m6809_ICount-= 9; break;
			case 0x18: illegal();  m6809_ICount-= 2; break;
			case 0x19: daa();	   m6809_ICount-= 2; break;
			case 0x1a: orcc();	   m6809_ICount-= 3; break;
			case 0x1b: illegal();  m6809_ICount-= 2; break;
			case 0x1c: andcc();    m6809_ICount-= 3; break;
			case 0x1d: sex();	   m6809_ICount-= 2; break;
			case 0x1e: exg();	   m6809_ICount-= 8; break;
			case 0x1f: tfr();	   m6809_ICount-= 6; break;
			case 0x20: bra();	   m6809_ICount-= 3; break;
			case 0x21: brn();	   m6809_ICount-= 3; break;
			case 0x22: bhi();	   m6809_ICount-= 3; break;
			case 0x23: bls();	   m6809_ICount-= 3; break;
			case 0x24: bcc();	   m6809_ICount-= 3; break;
			case 0x25: bcs();	   m6809_ICount-= 3; break;
			case 0x26: bne();	   m6809_ICount-= 3; break;
			case 0x27: beq();	   m6809_ICount-= 3; break;
			case 0x28: bvc();	   m6809_ICount-= 3; break;
			case 0x29: bvs();	   m6809_ICount-= 3; break;
			case 0x2a: bpl();	   m6809_ICount-= 3; break;
			case 0x2b: bmi();	   m6809_ICount-= 3; break;
			case 0x2c: bge();	   m6809_ICount-= 3; break;
			case 0x2d: blt();	   m6809_ICount-= 3; break;
			case 0x2e: bgt();	   m6809_ICount-= 3; break;
			case 0x2f: ble();	   m6809_ICount-= 3; break;
			case 0x30: leax();	   m6809_ICount-= 4; break;
			case 0x31: leay();	   m6809_ICount-= 4; break;
			case 0x32: leas();	   m6809_ICount-= 4; break;
			case 0x33: leau();	   m6809_ICount-= 4; break;
			case 0x34: pshs();	   m6809_ICount-= 5; break;
			case 0x35: puls();	   m6809_ICount-= 5; break;
			case 0x36: pshu();	   m6809_ICount-= 5; break;
			case 0x37: pulu();	   m6809_ICount-= 5; break;
			case 0x38: illegal();  m6809_ICount-= 2; break;
			case 0x39: rts();	   m6809_ICount-= 5; break;
			case 0x3a: abx();	   m6809_ICount-= 3; break;
			case 0x3b: rti();	   m6809_ICount-= 6; break;
			case 0x3c: cwai();	   m6809_ICount-=20; break;
			case 0x3d: mul();	   m6809_ICount-=11; break;
			case 0x3e: illegal();  m6809_ICount-= 2; break;
			case 0x3f: swi();	   m6809_ICount-=19; break;
			case 0x40: nega();	   m6809_ICount-= 2; break;
			case 0x41: illegal();  m6809_ICount-= 2; break;
			case 0x42: illegal();  m6809_ICount-= 2; break;
			case 0x43: coma();	   m6809_ICount-= 2; break;
			case 0x44: lsra();	   m6809_ICount-= 2; break;
			case 0x45: illegal();  m6809_ICount-= 2; break;
			case 0x46: rora();	   m6809_ICount-= 2; break;
			case 0x47: asra();	   m6809_ICount-= 2; break;
			case 0x48: asla();	   m6809_ICount-= 2; break;
			case 0x49: rola();	   m6809_ICount-= 2; break;
			case 0x4a: deca();	   m6809_ICount-= 2; break;
			case 0x4b: illegal();  m6809_ICount-= 2; break;
			case 0x4c: inca();	   m6809_ICount-= 2; break;
			case 0x4d: tsta();	   m6809_ICount-= 2; break;
			case 0x4e: illegal();  m6809_ICount-= 2; break;
			case 0x4f: clra();	   m6809_ICount-= 2; break;
			case 0x50: negb();	   m6809_ICount-= 2; break;
			case 0x51: illegal();  m6809_ICount-= 2; break;
			case 0x52: illegal();  m6809_ICount-= 2; break;
			case 0x53: comb();	   m6809_ICount-= 2; break;
			case 0x54: lsrb();	   m6809_ICount-= 2; break;
			case 0x55: illegal();  m6809_ICount-= 2; break;
			case 0x56: rorb();	   m6809_ICount-= 2; break;
			case 0x57: asrb();	   m6809_ICount-= 2; break;
			case 0x58: aslb();	   m6809_ICount-= 2; break;
			case 0x59: rolb();	   m6809_ICount-= 2; break;
			case 0x5a: decb();	   m6809_ICount-= 2; break;
			case 0x5b: illegal();  m6809_ICount-= 2; break;
			case 0x5c: incb();	   m6809_ICount-= 2; break;
			case 0x5d: tstb();	   m6809_ICount-= 2; break;
			case 0x5e: illegal();  m6809_ICount-= 2; break;
			case 0x5f: clrb();	   m6809_ICount-= 2; break;
			case 0x60: neg_ix();   m6809_ICount-= 6; break;
			case 0x61: illegal();  m6809_ICount-= 2; break;
			case 0x62: illegal();  m6809_ICount-= 2; break;
			case 0x63: com_ix();   m6809_ICount-= 6; break;
			case 0x64: lsr_ix();   m6809_ICount-= 6; break;
			case 0x65: illegal();  m6809_ICount-= 2; break;
			case 0x66: ror_ix();   m6809_ICount-= 6; break;
			case 0x67: asr_ix();   m6809_ICount-= 6; break;
			case 0x68: asl_ix();   m6809_ICount-= 6; break;
			case 0x69: rol_ix();   m6809_ICount-= 6; break;
			case 0x6a: dec_ix();   m6809_ICount-= 6; break;
			case 0x6b: illegal();  m6809_ICount-= 2; break;
			case 0x6c: inc_ix();   m6809_ICount-= 6; break;
			case 0x6d: tst_ix();   m6809_ICount-= 6; break;
			case 0x6e: jmp_ix();   m6809_ICount-= 3; break;
			case 0x6f: clr_ix();   m6809_ICount-= 6; break;
			case 0x70: neg_ex();   m6809_ICount-= 7; break;
			case 0x71: illegal();  m6809_ICount-= 2; break;
			case 0x72: illegal();  m6809_ICount-= 2; break;
			case 0x73: com_ex();   m6809_ICount-= 7; break;
			case 0x74: lsr_ex();   m6809_ICount-= 7; break;
			case 0x75: illegal();  m6809_ICount-= 2; break;
			case 0x76: ror_ex();   m6809_ICount-= 7; break;
			case 0x77: asr_ex();   m6809_ICount-= 7; break;
			case 0x78: asl_ex();   m6809_ICount-= 7; break;
			case 0x79: rol_ex();   m6809_ICount-= 7; break;
			case 0x7a: dec_ex();   m6809_ICount-= 7; break;
			case 0x7b: illegal();  m6809_ICount-= 2; break;
			case 0x7c: inc_ex();   m6809_ICount-= 7; break;
			case 0x7d: tst_ex();   m6809_ICount-= 7; break;
			case 0x7e: jmp_ex();   m6809_ICount-= 4; break;
			case 0x7f: clr_ex();   m6809_ICount-= 7; break;
			case 0x80: suba_im();  m6809_ICount-= 2; break;
			case 0x81: cmpa_im();  m6809_ICount-= 2; break;
			case 0x82: sbca_im();  m6809_ICount-= 2; break;
			case 0x83: subd_im();  m6809_ICount-= 4; break;
			case 0x84: anda_im();  m6809_ICount-= 2; break;
			case 0x85: bita_im();  m6809_ICount-= 2; break;
			case 0x86: lda_im();   m6809_ICount-= 2; break;
			case 0x87: sta_im();   m6809_ICount-= 2; break;
			case 0x88: eora_im();  m6809_ICount-= 2; break;
			case 0x89: adca_im();  m6809_ICount-= 2; break;
			case 0x8a: ora_im();   m6809_ICount-= 2; break;
			case 0x8b: adda_im();  m6809_ICount-= 2; break;
			case 0x8c: cmpx_im();  m6809_ICount-= 4; break;
			case 0x8d: bsr();	   m6809_ICount-= 7; break;
			case 0x8e: ldx_im();   m6809_ICount-= 3; break;
			case 0x8f: stx_im();   m6809_ICount-= 2; break;
			case 0x90: suba_di();  m6809_ICount-= 4; break;
			case 0x91: cmpa_di();  m6809_ICount-= 4; break;
			case 0x92: sbca_di();  m6809_ICount-= 4; break;
			case 0x93: subd_di();  m6809_ICount-= 6; break;
			case 0x94: anda_di();  m6809_ICount-= 4; break;
			case 0x95: bita_di();  m6809_ICount-= 4; break;
			case 0x96: lda_di();   m6809_ICount-= 4; break;
			case 0x97: sta_di();   m6809_ICount-= 4; break;
			case 0x98: eora_di();  m6809_ICount-= 4; break;
			case 0x99: adca_di();  m6809_ICount-= 4; break;
			case 0x9a: ora_di();   m6809_ICount-= 4; break;
			case 0x9b: adda_di();  m6809_ICount-= 4; break;
			case 0x9c: cmpx_di();  m6809_ICount-= 6; break;
			case 0x9d: jsr_di();   m6809_ICount-= 7; break;
			case 0x9e: ldx_di();   m6809_ICount-= 5; break;
			case 0x9f: stx_di();   m6809_ICount-= 5; break;
			case 0xa0: suba_ix();  m6809_ICount-= 4; break;
			case 0xa1: cmpa_ix();  m6809_ICount-= 4; break;
			case 0xa2: sbca_ix();  m6809_ICount-= 4; break;
			case 0xa3: subd_ix();  m6809_ICount-= 6; break;
			case 0xa4: anda_ix();  m6809_ICount-= 4; break;
			case 0xa5: bita_ix();  m6809_ICount-= 4; break;
			case 0xa6: lda_ix();   m6809_ICount-= 4; break;
			case 0xa7: sta_ix();   m6809_ICount-= 4; break;
			case 0xa8: eora_ix();  m6809_ICount-= 4; break;
			case 0xa9: adca_ix();  m6809_ICount-= 4; break;
			case 0xaa: ora_ix();   m6809_ICount-= 4; break;
			case 0xab: adda_ix();  m6809_ICount-= 4; break;
			case 0xac: cmpx_ix();  m6809_ICount-= 6; break;
			case 0xad: jsr_ix();   m6809_ICount-= 7; break;
			case 0xae: ldx_ix();   m6809_ICount-= 5; break;
			case 0xaf: stx_ix();   m6809_ICount-= 5; break;
			case 0xb0: suba_ex();  m6809_ICount-= 5; break;
			case 0xb1: cmpa_ex();  m6809_ICount-= 5; break;
			case 0xb2: sbca_ex();  m6809_ICount-= 5; break;
			case 0xb3: subd_ex();  m6809_ICount-= 7; break;
			case 0xb4: anda_ex();  m6809_ICount-= 5; break;
			case 0xb5: bita_ex();  m6809_ICount-= 5; break;
			case 0xb6: lda_ex();   m6809_ICount-= 5; break;
			case 0xb7: sta_ex();   m6809_ICount-= 5; break;
			case 0xb8: eora_ex();  m6809_ICount-= 5; break;
			case 0xb9: adca_ex();  m6809_ICount-= 5; break;
			case 0xba: ora_ex();   m6809_ICount-= 5; break;
			case 0xbb: adda_ex();  m6809_ICount-= 5; break;
			case 0xbc: cmpx_ex();  m6809_ICount-= 7; break;
			case 0xbd: jsr_ex();   m6809_ICount-= 8; break;
			case 0xbe: ldx_ex();   m6809_ICount-= 6; break;
			case 0xbf: stx_ex();   m6809_ICount-= 6; break;
			case 0xc0: subb_im();  m6809_ICount-= 2; break;
			case 0xc1: cmpb_im();  m6809_ICount-= 2; break;
			case 0xc2: sbcb_im();  m6809_ICount-= 2; break;
			case 0xc3: addd_im();  m6809_ICount-= 4; break;
			case 0xc4: andb_im();  m6809_ICount-= 2; break;
			case 0xc5: bitb_im();  m6809_ICount-= 2; break;
			case 0xc6: ldb_im();   m6809_ICount-= 2; break;
			case 0xc7: stb_im();   m6809_ICount-= 2; break;
			case 0xc8: eorb_im();  m6809_ICount-= 2; break;
			case 0xc9: adcb_im();  m6809_ICount-= 2; break;
			case 0xca: orb_im();   m6809_ICount-= 2; break;
			case 0xcb: addb_im();  m6809_ICount-= 2; break;
			case 0xcc: ldd_im();   m6809_ICount-= 3; break;
			case 0xcd: std_im();   m6809_ICount-= 2; break;
			case 0xce: ldu_im();   m6809_ICount-= 3; break;
			case 0xcf: stu_im();   m6809_ICount-= 3; break;
			case 0xd0: subb_di();  m6809_ICount-= 4; break;
			case 0xd1: cmpb_di();  m6809_ICount-= 4; break;
			case 0xd2: sbcb_di();  m6809_ICount-= 4; break;
			case 0xd3: addd_di();  m6809_ICount-= 6; break;
			case 0xd4: andb_di();  m6809_ICount-= 4; break;
			case 0xd5: bitb_di();  m6809_ICount-= 4; break;
			case 0xd6: ldb_di();   m6809_ICount-= 4; break;
			case 0xd7: stb_di();   m6809_ICount-= 4; break;
			case 0xd8: eorb_di();  m6809_ICount-= 4; break;
			case 0xd9: adcb_di();  m6809_ICount-= 4; break;
			case 0xda: orb_di();   m6809_ICount-= 4; break;
			case 0xdb: addb_di();  m6809_ICount-= 4; break;
			case 0xdc: ldd_di();   m6809_ICount-= 5; break;
			case 0xdd: std_di();   m6809_ICount-= 5; break;
			case 0xde: ldu_di();   m6809_ICount-= 5; break;
			case 0xdf: stu_di();   m6809_ICount-= 5; break;
			case 0xe0: subb_ix();  m6809_ICount-= 4; break;
			case 0xe1: cmpb_ix();  m6809_ICount-= 4; break;
			case 0xe2: sbcb_ix();  m6809_ICount-= 4; break;
			case 0xe3: addd_ix();  m6809_ICount-= 6; break;
			case 0xe4: andb_ix();  m6809_ICount-= 4; break;
			case 0xe5: bitb_ix();  m6809_ICount-= 4; break;
			case 0xe6: ldb_ix();   m6809_ICount-= 4; break;
			case 0xe7: stb_ix();   m6809_ICount-= 4; break;
			case 0xe8: eorb_ix();  m6809_ICount-= 4; break;
			case 0xe9: adcb_ix();  m6809_ICount-= 4; break;
			case 0xea: orb_ix();   m6809_ICount-= 4; break;
			case 0xeb: addb_ix();  m6809_ICount-= 4; break;
			case 0xec: ldd_ix();   m6809_ICount-= 5; break;
			case 0xed: std_ix();   m6809_ICount-= 5; break;
			case 0xee: ldu_ix();   m6809_ICount-= 5; break;
			case 0xef: stu_ix();   m6809_ICount-= 5; break;
			case 0xf0: subb_ex();  m6809_ICount-= 5; break;
			case 0xf1: cmpb_ex();  m6809_ICount-= 5; break;
			case 0xf2: sbcb_ex();  m6809_ICount-= 5; break;
			case 0xf3: addd_ex();  m6809_ICount-= 7; break;
			case 0xf4: andb_ex();  m6809_ICount-= 5; break;
			case 0xf5: bitb_ex();  m6809_ICount-= 5; break;
			case 0xf6: ldb_ex();   m6809_ICount-= 5; break;
			case 0xf7: stb_ex();   m6809_ICount-= 5; break;
			case 0xf8: eorb_ex();  m6809_ICount-= 5; break;
			case 0xf9: adcb_ex();  m6809_ICount-= 5; break;
			case 0xfa: orb_ex();   m6809_ICount-= 5; break;
			case 0xfb: addb_ex();  m6809_ICount-= 5; break;
			case 0xfc: ldd_ex();   m6809_ICount-= 6; break;
			case 0xfd: std_ex();   m6809_ICount-= 6; break;
			case 0xfe: ldu_ex();   m6809_ICount-= 6; break;
			case 0xff: stu_ex();   m6809_ICount-= 6; break;
			}
#else
            (*m6809_main[m6809.ireg])();
            m6809_ICount -= cycles1[m6809.ireg];
#endif

		} while( m6809_ICount > 0 );

        m6809_ICount -= m6809.extra_cycles;
		m6809.extra_cycles = 0;
    }

    return cycles - m6809_ICount;   /* NS 970908 */
}

M6809_INLINE void fetch_effective_address( void )
{
	UINT8 postbyte = ROP_ARG(PCD);
	PC++;

	switch(postbyte)
	{
	case 0x00: EA=X;												m6809_ICount-=1;   break;
	case 0x01: EA=X+1;												m6809_ICount-=1;   break;
	case 0x02: EA=X+2;												m6809_ICount-=1;   break;
	case 0x03: EA=X+3;												m6809_ICount-=1;   break;
	case 0x04: EA=X+4;												m6809_ICount-=1;   break;
	case 0x05: EA=X+5;												m6809_ICount-=1;   break;
	case 0x06: EA=X+6;												m6809_ICount-=1;   break;
	case 0x07: EA=X+7;												m6809_ICount-=1;   break;
	case 0x08: EA=X+8;												m6809_ICount-=1;   break;
	case 0x09: EA=X+9;												m6809_ICount-=1;   break;
	case 0x0a: EA=X+10; 											m6809_ICount-=1;   break;
	case 0x0b: EA=X+11; 											m6809_ICount-=1;   break;
	case 0x0c: EA=X+12; 											m6809_ICount-=1;   break;
	case 0x0d: EA=X+13; 											m6809_ICount-=1;   break;
	case 0x0e: EA=X+14; 											m6809_ICount-=1;   break;
	case 0x0f: EA=X+15; 											m6809_ICount-=1;   break;

	case 0x10: EA=X-16; 											m6809_ICount-=1;   break;
	case 0x11: EA=X-15; 											m6809_ICount-=1;   break;
	case 0x12: EA=X-14; 											m6809_ICount-=1;   break;
	case 0x13: EA=X-13; 											m6809_ICount-=1;   break;
	case 0x14: EA=X-12; 											m6809_ICount-=1;   break;
	case 0x15: EA=X-11; 											m6809_ICount-=1;   break;
	case 0x16: EA=X-10; 											m6809_ICount-=1;   break;
	case 0x17: EA=X-9;												m6809_ICount-=1;   break;
	case 0x18: EA=X-8;												m6809_ICount-=1;   break;
	case 0x19: EA=X-7;												m6809_ICount-=1;   break;
	case 0x1a: EA=X-6;												m6809_ICount-=1;   break;
	case 0x1b: EA=X-5;												m6809_ICount-=1;   break;
	case 0x1c: EA=X-4;												m6809_ICount-=1;   break;
	case 0x1d: EA=X-3;												m6809_ICount-=1;   break;
	case 0x1e: EA=X-2;												m6809_ICount-=1;   break;
	case 0x1f: EA=X-1;												m6809_ICount-=1;   break;

	case 0x20: EA=Y;												m6809_ICount-=1;   break;
	case 0x21: EA=Y+1;												m6809_ICount-=1;   break;
	case 0x22: EA=Y+2;												m6809_ICount-=1;   break;
	case 0x23: EA=Y+3;												m6809_ICount-=1;   break;
	case 0x24: EA=Y+4;												m6809_ICount-=1;   break;
	case 0x25: EA=Y+5;												m6809_ICount-=1;   break;
	case 0x26: EA=Y+6;												m6809_ICount-=1;   break;
	case 0x27: EA=Y+7;												m6809_ICount-=1;   break;
	case 0x28: EA=Y+8;												m6809_ICount-=1;   break;
	case 0x29: EA=Y+9;												m6809_ICount-=1;   break;
	case 0x2a: EA=Y+10; 											m6809_ICount-=1;   break;
	case 0x2b: EA=Y+11; 											m6809_ICount-=1;   break;
	case 0x2c: EA=Y+12; 											m6809_ICount-=1;   break;
	case 0x2d: EA=Y+13; 											m6809_ICount-=1;   break;
	case 0x2e: EA=Y+14; 											m6809_ICount-=1;   break;
	case 0x2f: EA=Y+15; 											m6809_ICount-=1;   break;

	case 0x30: EA=Y-16; 											m6809_ICount-=1;   break;
	case 0x31: EA=Y-15; 											m6809_ICount-=1;   break;
	case 0x32: EA=Y-14; 											m6809_ICount-=1;   break;
	case 0x33: EA=Y-13; 											m6809_ICount-=1;   break;
	case 0x34: EA=Y-12; 											m6809_ICount-=1;   break;
	case 0x35: EA=Y-11; 											m6809_ICount-=1;   break;
	case 0x36: EA=Y-10; 											m6809_ICount-=1;   break;
	case 0x37: EA=Y-9;												m6809_ICount-=1;   break;
	case 0x38: EA=Y-8;												m6809_ICount-=1;   break;
	case 0x39: EA=Y-7;												m6809_ICount-=1;   break;
	case 0x3a: EA=Y-6;												m6809_ICount-=1;   break;
	case 0x3b: EA=Y-5;												m6809_ICount-=1;   break;
	case 0x3c: EA=Y-4;												m6809_ICount-=1;   break;
	case 0x3d: EA=Y-3;												m6809_ICount-=1;   break;
	case 0x3e: EA=Y-2;												m6809_ICount-=1;   break;
	case 0x3f: EA=Y-1;												m6809_ICount-=1;   break;

	case 0x40: EA=U;												m6809_ICount-=1;   break;
	case 0x41: EA=U+1;												m6809_ICount-=1;   break;
	case 0x42: EA=U+2;												m6809_ICount-=1;   break;
	case 0x43: EA=U+3;												m6809_ICount-=1;   break;
	case 0x44: EA=U+4;												m6809_ICount-=1;   break;
	case 0x45: EA=U+5;												m6809_ICount-=1;   break;
	case 0x46: EA=U+6;												m6809_ICount-=1;   break;
	case 0x47: EA=U+7;												m6809_ICount-=1;   break;
	case 0x48: EA=U+8;												m6809_ICount-=1;   break;
	case 0x49: EA=U+9;												m6809_ICount-=1;   break;
	case 0x4a: EA=U+10; 											m6809_ICount-=1;   break;
	case 0x4b: EA=U+11; 											m6809_ICount-=1;   break;
	case 0x4c: EA=U+12; 											m6809_ICount-=1;   break;
	case 0x4d: EA=U+13; 											m6809_ICount-=1;   break;
	case 0x4e: EA=U+14; 											m6809_ICount-=1;   break;
	case 0x4f: EA=U+15; 											m6809_ICount-=1;   break;

	case 0x50: EA=U-16; 											m6809_ICount-=1;   break;
	case 0x51: EA=U-15; 											m6809_ICount-=1;   break;
	case 0x52: EA=U-14; 											m6809_ICount-=1;   break;
	case 0x53: EA=U-13; 											m6809_ICount-=1;   break;
	case 0x54: EA=U-12; 											m6809_ICount-=1;   break;
	case 0x55: EA=U-11; 											m6809_ICount-=1;   break;
	case 0x56: EA=U-10; 											m6809_ICount-=1;   break;
	case 0x57: EA=U-9;												m6809_ICount-=1;   break;
	case 0x58: EA=U-8;												m6809_ICount-=1;   break;
	case 0x59: EA=U-7;												m6809_ICount-=1;   break;
	case 0x5a: EA=U-6;												m6809_ICount-=1;   break;
	case 0x5b: EA=U-5;												m6809_ICount-=1;   break;
	case 0x5c: EA=U-4;												m6809_ICount-=1;   break;
	case 0x5d: EA=U-3;												m6809_ICount-=1;   break;
	case 0x5e: EA=U-2;												m6809_ICount-=1;   break;
	case 0x5f: EA=U-1;												m6809_ICount-=1;   break;

	case 0x60: EA=S;												m6809_ICount-=1;   break;
	case 0x61: EA=S+1;												m6809_ICount-=1;   break;
	case 0x62: EA=S+2;												m6809_ICount-=1;   break;
	case 0x63: EA=S+3;												m6809_ICount-=1;   break;
	case 0x64: EA=S+4;												m6809_ICount-=1;   break;
	case 0x65: EA=S+5;												m6809_ICount-=1;   break;
	case 0x66: EA=S+6;												m6809_ICount-=1;   break;
	case 0x67: EA=S+7;												m6809_ICount-=1;   break;
	case 0x68: EA=S+8;												m6809_ICount-=1;   break;
	case 0x69: EA=S+9;												m6809_ICount-=1;   break;
	case 0x6a: EA=S+10; 											m6809_ICount-=1;   break;
	case 0x6b: EA=S+11; 											m6809_ICount-=1;   break;
	case 0x6c: EA=S+12; 											m6809_ICount-=1;   break;
	case 0x6d: EA=S+13; 											m6809_ICount-=1;   break;
	case 0x6e: EA=S+14; 											m6809_ICount-=1;   break;
	case 0x6f: EA=S+15; 											m6809_ICount-=1;   break;

	case 0x70: EA=S-16; 											m6809_ICount-=1;   break;
	case 0x71: EA=S-15; 											m6809_ICount-=1;   break;
	case 0x72: EA=S-14; 											m6809_ICount-=1;   break;
	case 0x73: EA=S-13; 											m6809_ICount-=1;   break;
	case 0x74: EA=S-12; 											m6809_ICount-=1;   break;
	case 0x75: EA=S-11; 											m6809_ICount-=1;   break;
	case 0x76: EA=S-10; 											m6809_ICount-=1;   break;
	case 0x77: EA=S-9;												m6809_ICount-=1;   break;
	case 0x78: EA=S-8;												m6809_ICount-=1;   break;
	case 0x79: EA=S-7;												m6809_ICount-=1;   break;
	case 0x7a: EA=S-6;												m6809_ICount-=1;   break;
	case 0x7b: EA=S-5;												m6809_ICount-=1;   break;
	case 0x7c: EA=S-4;												m6809_ICount-=1;   break;
	case 0x7d: EA=S-3;												m6809_ICount-=1;   break;
	case 0x7e: EA=S-2;												m6809_ICount-=1;   break;
	case 0x7f: EA=S-1;												m6809_ICount-=1;   break;

	case 0x80: EA=X;	X++;										m6809_ICount-=2;   break;
	case 0x81: EA=X;	X+=2;										m6809_ICount-=3;   break;
	case 0x82: X--; 	EA=X;										m6809_ICount-=2;   break;
	case 0x83: X-=2;	EA=X;										m6809_ICount-=3;   break;
	case 0x84: EA=X;																   break;
	case 0x85: EA=X+SIGNED(B);										m6809_ICount-=1;   break;
	case 0x86: EA=X+SIGNED(A);										m6809_ICount-=1;   break;
	case 0x87: EA=0;																   break; /*   ILLEGAL*/
	case 0x88: IMMBYTE(EA); 	EA=X+SIGNED(EA);					m6809_ICount-=1;   break; /* this is a hack to make Vectrex work. It should be m6809_ICount-=1. Dunno where the cycle was lost :( */
	case 0x89: IMMWORD(ea); 	EA+=X;								m6809_ICount-=4;   break;
	case 0x8a: EA=0;																   break; /*   ILLEGAL*/
	case 0x8b: EA=X+D;												m6809_ICount-=4;   break;
	case 0x8c: IMMBYTE(EA); 	EA=PC+SIGNED(EA);					m6809_ICount-=1;   break;
	case 0x8d: IMMWORD(ea); 	EA+=PC; 							m6809_ICount-=5;   break;
	case 0x8e: EA=0;																   break; /*   ILLEGAL*/
	case 0x8f: IMMWORD(ea); 										m6809_ICount-=5;   break;

	case 0x90: EA=X;	X++;						EAD=RM16(EAD);	m6809_ICount-=5;   break; /* Indirect ,R+ not in my specs */
	case 0x91: EA=X;	X+=2;						EAD=RM16(EAD);	m6809_ICount-=6;   break;
	case 0x92: X--; 	EA=X;						EAD=RM16(EAD);	m6809_ICount-=5;   break;
	case 0x93: X-=2;	EA=X;						EAD=RM16(EAD);	m6809_ICount-=6;   break;
	case 0x94: EA=X;								EAD=RM16(EAD);	m6809_ICount-=3;   break;
	case 0x95: EA=X+SIGNED(B);						EAD=RM16(EAD);	m6809_ICount-=4;   break;
	case 0x96: EA=X+SIGNED(A);						EAD=RM16(EAD);	m6809_ICount-=4;   break;
	case 0x97: EA=0;																   break; /*   ILLEGAL*/
	case 0x98: IMMBYTE(EA); 	EA=X+SIGNED(EA);	EAD=RM16(EAD);	m6809_ICount-=4;   break;
	case 0x99: IMMWORD(ea); 	EA+=X;				EAD=RM16(EAD);	m6809_ICount-=7;   break;
	case 0x9a: EA=0;																   break; /*   ILLEGAL*/
	case 0x9b: EA=X+D;								EAD=RM16(EAD);	m6809_ICount-=7;   break;
	case 0x9c: IMMBYTE(EA); 	EA=PC+SIGNED(EA);	EAD=RM16(EAD);	m6809_ICount-=4;   break;
	case 0x9d: IMMWORD(ea); 	EA+=PC; 			EAD=RM16(EAD);	m6809_ICount-=8;   break;
	case 0x9e: EA=0;																   break; /*   ILLEGAL*/
	case 0x9f: IMMWORD(ea); 						EAD=RM16(EAD);	m6809_ICount-=8;   break;

	case 0xa0: EA=Y;	Y++;										m6809_ICount-=2;   break;
	case 0xa1: EA=Y;	Y+=2;										m6809_ICount-=3;   break;
	case 0xa2: Y--; 	EA=Y;										m6809_ICount-=2;   break;
	case 0xa3: Y-=2;	EA=Y;										m6809_ICount-=3;   break;
	case 0xa4: EA=Y;																   break;
	case 0xa5: EA=Y+SIGNED(B);										m6809_ICount-=1;   break;
	case 0xa6: EA=Y+SIGNED(A);										m6809_ICount-=1;   break;
	case 0xa7: EA=0;																   break; /*   ILLEGAL*/
	case 0xa8: IMMBYTE(EA); 	EA=Y+SIGNED(EA);					m6809_ICount-=1;   break;
	case 0xa9: IMMWORD(ea); 	EA+=Y;								m6809_ICount-=4;   break;
	case 0xaa: EA=0;																   break; /*   ILLEGAL*/
	case 0xab: EA=Y+D;												m6809_ICount-=4;   break;
	case 0xac: IMMBYTE(EA); 	EA=PC+SIGNED(EA);					m6809_ICount-=1;   break;
	case 0xad: IMMWORD(ea); 	EA+=PC; 							m6809_ICount-=5;   break;
	case 0xae: EA=0;																   break; /*   ILLEGAL*/
	case 0xaf: IMMWORD(ea); 										m6809_ICount-=5;   break;

	case 0xb0: EA=Y;	Y++;						EAD=RM16(EAD);	m6809_ICount-=5;   break;
	case 0xb1: EA=Y;	Y+=2;						EAD=RM16(EAD);	m6809_ICount-=6;   break;
	case 0xb2: Y--; 	EA=Y;						EAD=RM16(EAD);	m6809_ICount-=5;   break;
	case 0xb3: Y-=2;	EA=Y;						EAD=RM16(EAD);	m6809_ICount-=6;   break;
	case 0xb4: EA=Y;								EAD=RM16(EAD);	m6809_ICount-=3;   break;
	case 0xb5: EA=Y+SIGNED(B);						EAD=RM16(EAD);	m6809_ICount-=4;   break;
	case 0xb6: EA=Y+SIGNED(A);						EAD=RM16(EAD);	m6809_ICount-=4;   break;
	case 0xb7: EA=0;																   break; /*   ILLEGAL*/
	case 0xb8: IMMBYTE(EA); 	EA=Y+SIGNED(EA);	EAD=RM16(EAD);	m6809_ICount-=4;   break;
	case 0xb9: IMMWORD(ea); 	EA+=Y;				EAD=RM16(EAD);	m6809_ICount-=7;   break;
	case 0xba: EA=0;																   break; /*   ILLEGAL*/
	case 0xbb: EA=Y+D;								EAD=RM16(EAD);	m6809_ICount-=7;   break;
	case 0xbc: IMMBYTE(EA); 	EA=PC+SIGNED(EA);	EAD=RM16(EAD);	m6809_ICount-=4;   break;
	case 0xbd: IMMWORD(ea); 	EA+=PC; 			EAD=RM16(EAD);	m6809_ICount-=8;   break;
	case 0xbe: EA=0;																   break; /*   ILLEGAL*/
	case 0xbf: IMMWORD(ea); 						EAD=RM16(EAD);	m6809_ICount-=8;   break;

	case 0xc0: EA=U;			U++;								m6809_ICount-=2;   break;
	case 0xc1: EA=U;			U+=2;								m6809_ICount-=3;   break;
	case 0xc2: U--; 			EA=U;								m6809_ICount-=2;   break;
	case 0xc3: U-=2;			EA=U;								m6809_ICount-=3;   break;
	case 0xc4: EA=U;																   break;
	case 0xc5: EA=U+SIGNED(B);										m6809_ICount-=1;   break;
	case 0xc6: EA=U+SIGNED(A);										m6809_ICount-=1;   break;
	case 0xc7: EA=0;																   break; /*ILLEGAL*/
	case 0xc8: IMMBYTE(EA); 	EA=U+SIGNED(EA);					m6809_ICount-=1;   break;
	case 0xc9: IMMWORD(ea); 	EA+=U;								m6809_ICount-=4;   break;
	case 0xca: EA=0;																   break; /*ILLEGAL*/
	case 0xcb: EA=U+D;												m6809_ICount-=4;   break;
	case 0xcc: IMMBYTE(EA); 	EA=PC+SIGNED(EA);					m6809_ICount-=1;   break;
	case 0xcd: IMMWORD(ea); 	EA+=PC; 							m6809_ICount-=5;   break;
	case 0xce: EA=0;																   break; /*ILLEGAL*/
	case 0xcf: IMMWORD(ea); 										m6809_ICount-=5;   break;

	case 0xd0: EA=U;	U++;						EAD=RM16(EAD);	m6809_ICount-=5;   break;
	case 0xd1: EA=U;	U+=2;						EAD=RM16(EAD);	m6809_ICount-=6;   break;
	case 0xd2: U--; 	EA=U;						EAD=RM16(EAD);	m6809_ICount-=5;   break;
	case 0xd3: U-=2;	EA=U;						EAD=RM16(EAD);	m6809_ICount-=6;   break;
	case 0xd4: EA=U;								EAD=RM16(EAD);	m6809_ICount-=3;   break;
	case 0xd5: EA=U+SIGNED(B);						EAD=RM16(EAD);	m6809_ICount-=4;   break;
	case 0xd6: EA=U+SIGNED(A);						EAD=RM16(EAD);	m6809_ICount-=4;   break;
	case 0xd7: EA=0;																   break; /*ILLEGAL*/
	case 0xd8: IMMBYTE(EA); 	EA=U+SIGNED(EA);	EAD=RM16(EAD);	m6809_ICount-=4;   break;
	case 0xd9: IMMWORD(ea); 	EA+=U;				EAD=RM16(EAD);	m6809_ICount-=7;   break;
	case 0xda: EA=0;																   break; /*ILLEGAL*/
	case 0xdb: EA=U+D;								EAD=RM16(EAD);	m6809_ICount-=7;   break;
	case 0xdc: IMMBYTE(EA); 	EA=PC+SIGNED(EA);	EAD=RM16(EAD);	m6809_ICount-=4;   break;
	case 0xdd: IMMWORD(ea); 	EA+=PC; 			EAD=RM16(EAD);	m6809_ICount-=8;   break;
	case 0xde: EA=0;																   break; /*ILLEGAL*/
	case 0xdf: IMMWORD(ea); 						EAD=RM16(EAD);	m6809_ICount-=8;   break;

	case 0xe0: EA=S;	S++;										m6809_ICount-=2;   break;
	case 0xe1: EA=S;	S+=2;										m6809_ICount-=3;   break;
	case 0xe2: S--; 	EA=S;										m6809_ICount-=2;   break;
	case 0xe3: S-=2;	EA=S;										m6809_ICount-=3;   break;
	case 0xe4: EA=S;																   break;
	case 0xe5: EA=S+SIGNED(B);										m6809_ICount-=1;   break;
	case 0xe6: EA=S+SIGNED(A);										m6809_ICount-=1;   break;
	case 0xe7: EA=0;																   break; /*ILLEGAL*/
	case 0xe8: IMMBYTE(EA); 	EA=S+SIGNED(EA);					m6809_ICount-=1;   break;
	case 0xe9: IMMWORD(ea); 	EA+=S;								m6809_ICount-=4;   break;
	case 0xea: EA=0;																   break; /*ILLEGAL*/
	case 0xeb: EA=S+D;												m6809_ICount-=4;   break;
	case 0xec: IMMBYTE(EA); 	EA=PC+SIGNED(EA);					m6809_ICount-=1;   break;
	case 0xed: IMMWORD(ea); 	EA+=PC; 							m6809_ICount-=5;   break;
	case 0xee: EA=0;																   break;  /*ILLEGAL*/
	case 0xef: IMMWORD(ea); 										m6809_ICount-=5;   break;

	case 0xf0: EA=S;	S++;						EAD=RM16(EAD);	m6809_ICount-=5;   break;
	case 0xf1: EA=S;	S+=2;						EAD=RM16(EAD);	m6809_ICount-=6;   break;
	case 0xf2: S--; 	EA=S;						EAD=RM16(EAD);	m6809_ICount-=5;   break;
	case 0xf3: S-=2;	EA=S;						EAD=RM16(EAD);	m6809_ICount-=6;   break;
	case 0xf4: EA=S;								EAD=RM16(EAD);	m6809_ICount-=3;   break;
	case 0xf5: EA=S+SIGNED(B);						EAD=RM16(EAD);	m6809_ICount-=4;   break;
	case 0xf6: EA=S+SIGNED(A);						EAD=RM16(EAD);	m6809_ICount-=4;   break;
	case 0xf7: EA=0;																   break; /*ILLEGAL*/
	case 0xf8: IMMBYTE(EA); 	EA=S+SIGNED(EA);	EAD=RM16(EAD);	m6809_ICount-=4;   break;
	case 0xf9: IMMWORD(ea); 	EA+=S;				EAD=RM16(EAD);	m6809_ICount-=7;   break;
	case 0xfa: EA=0;																   break; /*ILLEGAL*/
	case 0xfb: EA=S+D;								EAD=RM16(EAD);	m6809_ICount-=7;   break;
	case 0xfc: IMMBYTE(EA); 	EA=PC+SIGNED(EA);	EAD=RM16(EAD);	m6809_ICount-=4;   break;
	case 0xfd: IMMWORD(ea); 	EA+=PC; 			EAD=RM16(EAD);	m6809_ICount-=8;   break;
	case 0xfe: EA=0;																   break; /*ILLEGAL*/
	case 0xff: IMMWORD(ea); 						EAD=RM16(EAD);	m6809_ICount-=8;   break;
	}
}

#if 0

/**************************************************************************
 * Generic set_info
 **************************************************************************/

static void m6809_set_info(UINT32 state, cpuinfo *info)
{
	switch (state)
	{
		/* --- the following bits of info are set as 64-bit signed integers --- */
		case CPUINFO_INT_INPUT_STATE + M6809_IRQ_LINE:	set_irq_line(M6809_IRQ_LINE, info->i);	break;
		case CPUINFO_INT_INPUT_STATE + M6809_FIRQ_LINE:	set_irq_line(M6809_FIRQ_LINE, info->i); break;
		case CPUINFO_INT_INPUT_STATE + M6809_INPUT_LINE_NMI:	set_irq_line(M6809_INPUT_LINE_NMI, info->i);	break;

		case CPUINFO_INT_PC:
		case CPUINFO_INT_REGISTER + M6809_PC:			PC = info->i; CHANGE_PC;				break;
		case CPUINFO_INT_SP:
		case CPUINFO_INT_REGISTER + M6809_S:			S = info->i;							break;
		case CPUINFO_INT_REGISTER + M6809_CC:			CC = info->i; CHECK_IRQ_LINES;			break;
		case CPUINFO_INT_REGISTER + M6809_U:			U = info->i;							break;
		case CPUINFO_INT_REGISTER + M6809_A:			A = info->i;							break;
		case CPUINFO_INT_REGISTER + M6809_B:			B = info->i;							break;
		case CPUINFO_INT_REGISTER + M6809_X:			X = info->i;							break;
		case CPUINFO_INT_REGISTER + M6809_Y:			Y = info->i;							break;
		case CPUINFO_INT_REGISTER + M6809_DP:			DP = info->i;							break;
	}
}



/**************************************************************************
 * Generic get_info
 **************************************************************************/

void m6809_get_info(UINT32 state, cpuinfo *info)
{
	switch (state)
	{
		/* --- the following bits of info are returned as 64-bit signed integers --- */
		case CPUINFO_INT_CONTEXT_SIZE:					info->i = sizeof(m6809);				break;
		case CPUINFO_INT_INPUT_LINES:					info->i = 2;							break;
		case CPUINFO_INT_DEFAULT_IRQ_VECTOR:			info->i = 0;							break;
		case CPUINFO_INT_ENDIANNESS:					info->i = CPU_IS_BE;					break;
		case CPUINFO_INT_CLOCK_MULTIPLIER:				info->i = 1;							break;
		case CPUINFO_INT_CLOCK_DIVIDER:					info->i = 1;							break;
		case CPUINFO_INT_MIN_INSTRUCTION_BYTES:			info->i = 1;							break;
		case CPUINFO_INT_MAX_INSTRUCTION_BYTES:			info->i = 5;							break;
		case CPUINFO_INT_MIN_CYCLES:					info->i = 2;							break;
		case CPUINFO_INT_MAX_CYCLES:					info->i = 19;							break;

		case CPUINFO_INT_DATABUS_WIDTH + ADDRESS_SPACE_PROGRAM:	info->i = 8;					break;
		case CPUINFO_INT_ADDRBUS_WIDTH + ADDRESS_SPACE_PROGRAM: info->i = 16;					break;
		case CPUINFO_INT_ADDRBUS_SHIFT + ADDRESS_SPACE_PROGRAM: info->i = 0;					break;
		case CPUINFO_INT_DATABUS_WIDTH + ADDRESS_SPACE_DATA:	info->i = 0;					break;
		case CPUINFO_INT_ADDRBUS_WIDTH + ADDRESS_SPACE_DATA: 	info->i = 0;					break;
		case CPUINFO_INT_ADDRBUS_SHIFT + ADDRESS_SPACE_DATA: 	info->i = 0;					break;
		case CPUINFO_INT_DATABUS_WIDTH + ADDRESS_SPACE_IO:		info->i = 0;					break;
		case CPUINFO_INT_ADDRBUS_WIDTH + ADDRESS_SPACE_IO: 		info->i = 0;					break;
		case CPUINFO_INT_ADDRBUS_SHIFT + ADDRESS_SPACE_IO: 		info->i = 0;					break;

		case CPUINFO_INT_INPUT_STATE + M6809_IRQ_LINE:	info->i = m6809.irq_state[M6809_IRQ_LINE]; break;
		case CPUINFO_INT_INPUT_STATE + M6809_FIRQ_LINE:	info->i = m6809.irq_state[M6809_FIRQ_LINE]; break;
		case CPUINFO_INT_INPUT_STATE + M6809_INPUT_LINE_NMI:	info->i = m6809.nmi_state;				break;

		case CPUINFO_INT_PREVIOUSPC:					info->i = wPPC;							break;

		case CPUINFO_INT_PC:
		case CPUINFO_INT_REGISTER + M6809_PC:			info->i = PC;							break;
		case CPUINFO_INT_SP:
		case CPUINFO_INT_REGISTER + M6809_S:			info->i = S;							break;
		case CPUINFO_INT_REGISTER + M6809_CC:			info->i = CC;							break;
		case CPUINFO_INT_REGISTER + M6809_U:			info->i = U;							break;
		case CPUINFO_INT_REGISTER + M6809_A:			info->i = A;							break;
		case CPUINFO_INT_REGISTER + M6809_B:			info->i = B;							break;
		case CPUINFO_INT_REGISTER + M6809_X:			info->i = X;							break;
		case CPUINFO_INT_REGISTER + M6809_Y:			info->i = Y;							break;
		case CPUINFO_INT_REGISTER + M6809_DP:			info->i = DP;							break;

		/* --- the following bits of info are returned as pointers to data or functions --- */
		case CPUINFO_PTR_SET_INFO:						info->setinfo = m6809_set_info;			break;
		case CPUINFO_PTR_GET_CONTEXT:					info->getcontext = m6809_get_context;	break;
		case CPUINFO_PTR_SET_CONTEXT:					info->setcontext = m6809_set_context;	break;
		case CPUINFO_PTR_INIT:							info->init = m6809_init;				break;
		case CPUINFO_PTR_RESET:							info->reset = m6809_reset;				break;
		case CPUINFO_PTR_EXIT:							info->exit = m6809_exit;				break;
		case CPUINFO_PTR_EXECUTE:						info->execute = m6809_execute;			break;
		case CPUINFO_PTR_BURN:							info->burn = NULL;						break;
		case CPUINFO_PTR_DISASSEMBLE:					info->disassemble = m6809_dasm;			break;
		case CPUINFO_PTR_INSTRUCTION_COUNTER:			info->icount = &m6809_ICount;			break;

		/* --- the following bits of info are returned as NULL-terminated strings --- */
		case CPUINFO_STR_NAME:							strcpy(info->s, "M6809");				break;
		case CPUINFO_STR_CORE_FAMILY:					strcpy(info->s, "Motorola 6809");		break;
		case CPUINFO_STR_CORE_VERSION:					strcpy(info->s, "1.11");				break;
		case CPUINFO_STR_CORE_FILE:						strcpy(info->s, __FILE__);				break;
		case CPUINFO_STR_CORE_CREDITS:					strcpy(info->s, "Copyright John Butler"); break;

		case CPUINFO_STR_FLAGS:
			sprintf(info->s, "%c%c%c%c%c%c%c%c",
				m6809.cc & 0x80 ? 'E':'.',
				m6809.cc & 0x40 ? 'F':'.',
                m6809.cc & 0x20 ? 'H':'.',
                m6809.cc & 0x10 ? 'I':'.',
                m6809.cc & 0x08 ? 'N':'.',
                m6809.cc & 0x04 ? 'Z':'.',
                m6809.cc & 0x02 ? 'V':'.',
                m6809.cc & 0x01 ? 'C':'.');
            break;

		case CPUINFO_STR_REGISTER + M6809_PC:			sprintf(info->s, "PC:%04X", m6809.pc.w.l); break;
		case CPUINFO_STR_REGISTER + M6809_S:			sprintf(info->s, "S:%04X", m6809.s.w.l); break;
		case CPUINFO_STR_REGISTER + M6809_CC:			sprintf(info->s, "CC:%02X", m6809.cc); break;
		case CPUINFO_STR_REGISTER + M6809_U:			sprintf(info->s, "U:%04X", m6809.u.w.l); break;
		case CPUINFO_STR_REGISTER + M6809_A:			sprintf(info->s, "A:%02X", m6809.d.b.h); break;
		case CPUINFO_STR_REGISTER + M6809_B:			sprintf(info->s, "B:%02X", m6809.d.b.l); break;
		case CPUINFO_STR_REGISTER + M6809_X:			sprintf(info->s, "X:%04X", m6809.x.w.l); break;
		case CPUINFO_STR_REGISTER + M6809_Y:			sprintf(info->s, "Y:%04X", m6809.y.w.l); break;
		case CPUINFO_STR_REGISTER + M6809_DP:			sprintf(info->s, "DP:%02X", m6809.dp.b.h); break;
	}
}


/**************************************************************************
 * CPU-specific set_info
 **************************************************************************/

void m6809e_get_info(UINT32 state, cpuinfo *info)
{
	switch (state)
	{
		/* --- the following bits of info are returned as 64-bit signed integers --- */
		case CPUINFO_INT_CLOCK_MULTIPLIER:				info->i = 1;							break;
		case CPUINFO_INT_CLOCK_DIVIDER:					info->i = 4;							break;

		/* --- the following bits of info are returned as NULL-terminated strings --- */
		case CPUINFO_STR_NAME:							strcpy(info->s, "M6809E");				break;

		default:										m6809_get_info(state, info);			break;
	}
}

#endif
