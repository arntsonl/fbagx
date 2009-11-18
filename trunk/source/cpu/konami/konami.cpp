/*** konami: Portable Konami cpu emulator ******************************************

    Copyright Nicola Salmoria and the MAME Team

    Based on M6809 cpu core copyright John Butler

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
991022 HJB:
    Tried to improve speed: Using bit7 of cycles1 as flag for multi
    byte opcodes is gone, those opcodes now instead go through opcode2().
    KONAMI_INLINEd fetch_effective_address() into that function as well.
    Got rid of the slow/fast flags for stack (S and U) memory accesses.
    Minor changes to use 32 bit values as arguments to memory functions
    and added defines for that purpose (e.g. X = 16bit XD = 32bit).

990720 EHC:
    Created this file

*****************************************************************************/

#include "burnint.h"
#include "konami.h"
#include "konami_intf.h"
#define VERBOSE 0

#define change_pc(x)	PC=x
#define logerror printf

#define LOG(x)	do { if (VERBOSE) logerror x; } while (0)

#define KONAMI_INLINE	inline

/* Konami Registers */
typedef struct
{
	PAIR	pc; 		/* Program counter */
	PAIR    ppc;        /* Previous program counter */
	PAIR    d;          /* Accumulator a and b */
	PAIR    dp;         /* Direct Page register (page in MSB) */
	PAIR	u, s;		/* Stack pointers */
	PAIR	x, y;		/* Index registers */
	UINT8   cc;
	UINT8	ireg;		/* first opcode */
	UINT8   irq_state[2];
	int     extra_cycles; /* cycles used up by interrupts */
	UINT8   int_state;  /* SYNC and CWAI flags */
	UINT8	nmi_state;
	int	nTotalCycles;
	int     (*irq_callback)(int irqline);
	void 	(*setlines_callback)( int lines ); /* callback called when A16-A23 are set */
} konami_Regs;

/* flag bits in the cc register */
#define CC_C    0x01        /* Carry */
#define CC_V    0x02        /* Overflow */
#define CC_Z    0x04        /* Zero */
#define CC_N    0x08        /* Negative */
#define CC_II   0x10        /* Inhibit IRQ */
#define CC_H    0x20        /* Half (auxiliary) carry */
#define CC_IF   0x40        /* Inhibit FIRQ */
#define CC_E    0x80        /* entire state pushed */

/* Konami registers */
static konami_Regs konami;

#define	pPPC    konami.ppc
#define pPC 	konami.pc
#define pU		konami.u
#define pS		konami.s
#define pX		konami.x
#define pY		konami.y
#define pD		konami.d

#define	wPPC	konami.ppc.w.l
#define PC  	konami.pc.w.l
#define PCD 	konami.pc.d
#define U		konami.u.w.l
#define UD		konami.u.d
#define S		konami.s.w.l
#define SD		konami.s.d
#define X		konami.x.w.l
#define XD		konami.x.d
#define Y		konami.y.w.l
#define YD		konami.y.d
#define D   	konami.d.w.l
#define A   	konami.d.b.h
#define B		konami.d.b.l
#define DP		konami.dp.b.h
#define DPD 	konami.dp.d
#define CC  	konami.cc

static PAIR ea;         /* effective address */
#define EA	ea.w.l
#define EAD ea.d

#define KONAMI_CWAI		8	/* set when CWAI is waiting for an interrupt */
#define KONAMI_SYNC		16	/* set when SYNC is waiting for an interrupt */
#define KONAMI_LDS		32	/* set when LDS occured at least once */

static int nCyclesToDo = 0;

#define CHECK_IRQ_LINES 												\
	if( konami.irq_state[KONAMI_IRQ_LINE] != KONAMI_CLEAR_LINE ||				\
		konami.irq_state[KONAMI_FIRQ_LINE] != KONAMI_CLEAR_LINE )				\
		konami.int_state &= ~KONAMI_SYNC; /* clear SYNC flag */			\
	if( konami.irq_state[KONAMI_FIRQ_LINE]!=KONAMI_CLEAR_LINE && !(CC & CC_IF) ) \
	{																	\
		/* fast IRQ */													\
		/* state already saved by CWAI? */								\
		if( konami.int_state & KONAMI_CWAI )							\
		{																\
			konami.int_state &= ~KONAMI_CWAI;  /* clear CWAI */			\
			konami.extra_cycles += 7;		 /* subtract +7 cycles */	\
        }                                                               \
		else															\
		{																\
			CC &= ~CC_E;				/* save 'short' state */        \
			PUSHWORD(pPC);												\
			PUSHBYTE(CC);												\
			konami.extra_cycles += 10;	/* subtract +10 cycles */		\
		}																\
		CC |= CC_IF | CC_II;			/* inhibit FIRQ and IRQ */		\
		PCD = RM16(0xfff6); 											\
		change_pc(PC);					/* TS 971002 */ 				\
		(void)(*konami.irq_callback)(KONAMI_FIRQ_LINE);					\
	}																	\
	else																\
	if( konami.irq_state[KONAMI_IRQ_LINE]!=KONAMI_CLEAR_LINE && !(CC & CC_II) )\
	{																	\
		/* standard IRQ */												\
		/* state already saved by CWAI? */								\
		if( konami.int_state & KONAMI_CWAI )							\
		{																\
			konami.int_state &= ~KONAMI_CWAI;  /* clear CWAI flag */	\
			konami.extra_cycles += 7;		 /* subtract +7 cycles */	\
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
			konami.extra_cycles += 19;	 /* subtract +19 cycles */		\
		}																\
		CC |= CC_II;					/* inhibit IRQ */				\
		PCD = RM16(0xfff8); 											\
		change_pc(PC);					/* TS 971002 */ 				\
		(void)(*konami.irq_callback)(KONAMI_IRQ_LINE);					\
	}

/* public globals */
static int konami_ICount;
//int konami_Flags; /* flags for speed optimization (obsolete!!) */

/* these are re-defined in konami.h TO RAM, ROM or functions in memory.c */
#define RM(Addr)			KONAMI_RDMEM(Addr)
#define WM(Addr,Value)		KONAMI_WRMEM(Addr,Value)
#define ROP(Addr)			KONAMI_RDOP(Addr)
#define ROP_ARG(Addr)		KONAMI_RDOP_ARG(Addr)

#define SIGNED(a)	(UINT16)(INT16)(INT8)(a)

/* macros to access memory */
#define IMMBYTE(b)	{ b = ROP_ARG(PCD); PC++; }
#define IMMWORD(w)	{ w.d = (ROP_ARG(PCD)<<8) | ROP_ARG(PCD+1); PC += 2; }

#define PUSHBYTE(b) --S; WM(SD,b)
#define PUSHWORD(w) --S; WM(SD,w.b.l); --S; WM(SD,w.b.h)
#define PULLBYTE(b) b=KONAMI_RDMEM(SD); S++
#define PULLWORD(w) w=KONAMI_RDMEM(SD)<<8; S++; w|=KONAMI_RDMEM(SD); S++

#define PSHUBYTE(b) --U; WM(UD,b);
#define PSHUWORD(w) --U; WM(UD,w.b.l); --U; WM(UD,w.b.h)
#define PULUBYTE(b) b=KONAMI_RDMEM(UD); U++
#define PULUWORD(w) w=KONAMI_RDMEM(UD)<<8; U++; w|=KONAMI_RDMEM(UD); U++

#define CLR_HNZVC	CC&=~(CC_H|CC_N|CC_Z|CC_V|CC_C)
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
#define DIRBYTE(b) DIRECT; b=RM(EAD)
#define DIRWORD(w) DIRECT; w.d=RM16(EAD)
#define EXTBYTE(b) EXTENDED; b=RM(EAD)
#define EXTWORD(w) EXTENDED; w.d=RM16(EAD)

/* macros for branch instructions */
#define BRANCH(f) { 					\
	UINT8 t;							\
	IMMBYTE(t); 						\
	if( f ) 							\
	{									\
		PC += SIGNED(t);				\
		change_pc(PC);	/* TS 971002 */ \
	}									\
}

#define LBRANCH(f) {                    \
	PAIR t; 							\
	IMMWORD(t); 						\
	if( f ) 							\
	{									\
		konami_ICount -= 1;				\
		PC += t.w.l;					\
		change_pc(PC);	/* TS 971002 */ \
	}									\
}

#define NXORV  ((CC&CC_N)^((CC&CC_V)<<2))

/* macros for setting/getting registers in TFR/EXG instructions */
#define GETREG(val,reg) 				\
	switch(reg) {						\
	case 0: val = A;	break;			\
	case 1: val = B; 	break; 			\
	case 2: val = X; 	break;			\
	case 3: val = Y;	break; 			\
	case 4: val = S; 	break; /* ? */	\
	case 5: val = U;	break;			\
	default: val = 0xff; logerror("Unknown TFR/EXG idx at PC:%04x\n", PC ); break; \
}

#define SETREG(val,reg) 				\
	switch(reg) {						\
	case 0: A = val;	break;			\
	case 1: B = val;	break;			\
	case 2: X = val; 	break;			\
	case 3: Y = val;	break;			\
	case 4: S = val;	break; /* ? */	\
	case 5: U = val; 	break;			\
	default: logerror("Unknown TFR/EXG idx at PC:%04x\n", PC ); break; \
}

/* opcode timings */
static const UINT8 cycles1[] =
{
	/*   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F */
  /*0*/  1, 1, 1, 1, 1, 1, 1, 1, 4, 4, 4, 4, 5, 5, 5, 5,
  /*1*/  2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
  /*2*/  2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
  /*3*/  2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 7, 6,
  /*4*/  3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 3, 3, 4, 4,
  /*5*/  4, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 1, 1, 1,
  /*6*/  3, 3, 3, 3, 3, 3, 3, 3, 5, 5, 5, 5, 5, 5, 5, 5,
  /*7*/  3, 3, 3, 3, 3, 3, 3, 3, 5, 5, 5, 5, 5, 5, 5, 5,
  /*8*/  2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 5,
  /*9*/  2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 6,
  /*A*/  2, 2, 2, 4, 4, 4, 4, 4, 2, 2, 2, 2, 3, 3, 2, 1,
  /*B*/  3, 2, 2,11,22,11, 2, 4, 3, 3, 3, 3, 3, 3, 3, 3,
  /*C*/  3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 3, 2,
  /*D*/  2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
  /*E*/  1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
  /*F*/  1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1
};

KONAMI_INLINE UINT32 RM16( UINT32 Addr )
{
	UINT32 result = RM(Addr) << 8;
	return result | RM((Addr+1)&0xffff);
}

KONAMI_INLINE void WM16( UINT32 Addr, PAIR *p )
{
	WM( Addr, p->b.h );
	WM( (Addr+1)&0xffff, p->b.l );
}

/****************************************************************************
 * Get all registers in given buffer
 ****************************************************************************/
#if 0
static void konami_get_context(void *dst)
{
	if( dst )
		*(konami_Regs*)dst = konami;
}
#endif

/****************************************************************************
 * Set all registers to given values
 ****************************************************************************/
#if 0
static void konami_set_context(void *src)
{
	if( src )
		konami = *(konami_Regs*)src;
    change_pc(PC);    /* TS 971002 */

    CHECK_IRQ_LINES;
}
#endif

/****************************************************************************/
/* Reset registers to their initial values                                  */
/****************************************************************************/
void konami_init(int (*irqcallback)(int))
{
	konami.irq_callback = irqcallback;
}

void konamiReset()
{
	konami.nTotalCycles = 0;
	konami.int_state = 0;
	konami.nmi_state = KONAMI_CLEAR_LINE;
	konami.irq_state[0] = KONAMI_CLEAR_LINE;
	konami.irq_state[1] = KONAMI_CLEAR_LINE;

	DPD = 0;			/* Reset direct page register */

	CC |= CC_II;        /* IRQ disabled */
	CC |= CC_IF;        /* FIRQ disabled */

	PCD = RM16(0xfffe);
	change_pc(PC);    /* TS 971002 */
}

#if 0
static void konami_exit(void)
{
}
#endif

/* Generate interrupts */
/****************************************************************************
 * Set IRQ line state
 ****************************************************************************/
void konami_set_irq_line(int irqline, int state)
{
	if (irqline == KONAMI_INPUT_LINE_NMI)
	{
		if (konami.nmi_state == state) return;
		konami.nmi_state = state;
	//	LOG(("KONAMI#%d set_nmi_line %d\n", cpu_getactivecpu(), state));
		if( state == KONAMI_CLEAR_LINE ) return;

		/* if the stack was not yet initialized */
	    if( !(konami.int_state & KONAMI_LDS) ) return;

	    konami.int_state &= ~KONAMI_SYNC;
		/* state already saved by CWAI? */
		if( konami.int_state & KONAMI_CWAI )
		{
			konami.int_state &= ~KONAMI_CWAI;
			konami.extra_cycles += 7;	/* subtract +7 cycles next time */
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
			konami.extra_cycles += 19;	/* subtract +19 cycles next time */
		}
		CC |= CC_IF | CC_II;			/* inhibit FIRQ and IRQ */
		PCD = RM16(0xfffc);
		change_pc(PC);					/* TS 971002 */
	}
	else if (irqline < 2)
	{
	//	LOG(("KONAMI#%d set_irq_line %d, %d\n", cpu_getactivecpu(), irqline, state));
		konami.irq_state[irqline] = state;
		if (state == KONAMI_CLEAR_LINE) return;
		CHECK_IRQ_LINES;
	}
}

#ifndef __cplusplus
extern "C" {
#endif

KONAMI_INLINE void abx(void);
KONAMI_INLINE void adca_di(void);
KONAMI_INLINE void adca_ex(void);
KONAMI_INLINE void adca_im(void);
KONAMI_INLINE void adca_ix(void);
KONAMI_INLINE void adcb_di(void);
KONAMI_INLINE void adcb_ex(void);
KONAMI_INLINE void adcb_im(void);
KONAMI_INLINE void adcb_ix(void);
KONAMI_INLINE void adda_di(void);
KONAMI_INLINE void adda_ex(void);
KONAMI_INLINE void adda_im(void);
KONAMI_INLINE void adda_ix(void);
KONAMI_INLINE void addb_di(void);
KONAMI_INLINE void addb_ex(void);
KONAMI_INLINE void addb_im(void);
KONAMI_INLINE void addb_ix(void);
KONAMI_INLINE void addd_di(void);
KONAMI_INLINE void addd_ex(void);
KONAMI_INLINE void addd_im(void);
KONAMI_INLINE void addd_ix(void);
KONAMI_INLINE void anda_di(void);
KONAMI_INLINE void anda_ex(void);
KONAMI_INLINE void anda_im(void);
KONAMI_INLINE void anda_ix(void);
KONAMI_INLINE void andb_di(void);
KONAMI_INLINE void andb_ex(void);
KONAMI_INLINE void andb_im(void);
KONAMI_INLINE void andb_ix(void);
KONAMI_INLINE void andcc(void);
KONAMI_INLINE void asl_di(void);
KONAMI_INLINE void asl_ex(void);
KONAMI_INLINE void asl_ix(void);
KONAMI_INLINE void asla(void);
KONAMI_INLINE void aslb(void);
KONAMI_INLINE void asr_di(void);
KONAMI_INLINE void asr_ex(void);
KONAMI_INLINE void asr_ix(void);
KONAMI_INLINE void asra(void);
KONAMI_INLINE void asrb(void);
KONAMI_INLINE void bcc(void);
KONAMI_INLINE void bcs(void);
KONAMI_INLINE void beq(void);
KONAMI_INLINE void bge(void);
KONAMI_INLINE void bgt(void);
KONAMI_INLINE void bhi(void);
KONAMI_INLINE void bita_di(void);
KONAMI_INLINE void bita_ex(void);
KONAMI_INLINE void bita_im(void);
KONAMI_INLINE void bita_ix(void);
KONAMI_INLINE void bitb_di(void);
KONAMI_INLINE void bitb_ex(void);
KONAMI_INLINE void bitb_im(void);
KONAMI_INLINE void bitb_ix(void);
KONAMI_INLINE void ble(void);
KONAMI_INLINE void bls(void);
KONAMI_INLINE void blt(void);
KONAMI_INLINE void bmi(void);
KONAMI_INLINE void bne(void);
KONAMI_INLINE void bpl(void);
KONAMI_INLINE void bra(void);
KONAMI_INLINE void brn(void);
KONAMI_INLINE void bsr(void);
KONAMI_INLINE void bvc(void);
KONAMI_INLINE void bvs(void);
KONAMI_INLINE void clr_di(void);
KONAMI_INLINE void clr_ex(void);
KONAMI_INLINE void clr_ix(void);
KONAMI_INLINE void clra(void);
KONAMI_INLINE void clrb(void);
KONAMI_INLINE void cmpa_di(void);
KONAMI_INLINE void cmpa_ex(void);
KONAMI_INLINE void cmpa_im(void);
KONAMI_INLINE void cmpa_ix(void);
KONAMI_INLINE void cmpb_di(void);
KONAMI_INLINE void cmpb_ex(void);
KONAMI_INLINE void cmpb_im(void);
KONAMI_INLINE void cmpb_ix(void);
KONAMI_INLINE void cmpd_di(void);
KONAMI_INLINE void cmpd_ex(void);
KONAMI_INLINE void cmpd_im(void);
KONAMI_INLINE void cmpd_ix(void);
KONAMI_INLINE void cmps_di(void);
KONAMI_INLINE void cmps_ex(void);
KONAMI_INLINE void cmps_im(void);
KONAMI_INLINE void cmps_ix(void);
KONAMI_INLINE void cmpu_di(void);
KONAMI_INLINE void cmpu_ex(void);
KONAMI_INLINE void cmpu_im(void);
KONAMI_INLINE void cmpu_ix(void);
KONAMI_INLINE void cmpx_di(void);
KONAMI_INLINE void cmpx_ex(void);
KONAMI_INLINE void cmpx_im(void);
KONAMI_INLINE void cmpx_ix(void);
KONAMI_INLINE void cmpy_di(void);
KONAMI_INLINE void cmpy_ex(void);
KONAMI_INLINE void cmpy_im(void);
KONAMI_INLINE void cmpy_ix(void);
KONAMI_INLINE void com_di(void);
KONAMI_INLINE void com_ex(void);
KONAMI_INLINE void com_ix(void);
KONAMI_INLINE void coma(void);
KONAMI_INLINE void comb(void);
KONAMI_INLINE void cwai(void);
KONAMI_INLINE void daa(void);
KONAMI_INLINE void dec_di(void);
KONAMI_INLINE void dec_ex(void);
KONAMI_INLINE void dec_ix(void);
KONAMI_INLINE void deca(void);
KONAMI_INLINE void decb(void);
KONAMI_INLINE void eora_di(void);
KONAMI_INLINE void eora_ex(void);
KONAMI_INLINE void eora_im(void);
KONAMI_INLINE void eora_ix(void);
KONAMI_INLINE void eorb_di(void);
KONAMI_INLINE void eorb_ex(void);
KONAMI_INLINE void eorb_im(void);
KONAMI_INLINE void eorb_ix(void);
KONAMI_INLINE void exg(void);
KONAMI_INLINE void illegal(void);
KONAMI_INLINE void inc_di(void);
KONAMI_INLINE void inc_ex(void);
KONAMI_INLINE void inc_ix(void);
KONAMI_INLINE void inca(void);
KONAMI_INLINE void incb(void);
KONAMI_INLINE void jmp_di(void);
KONAMI_INLINE void jmp_ex(void);
KONAMI_INLINE void jmp_ix(void);
KONAMI_INLINE void jsr_di(void);
KONAMI_INLINE void jsr_ex(void);
KONAMI_INLINE void jsr_ix(void);
KONAMI_INLINE void lbcc(void);
KONAMI_INLINE void lbcs(void);
KONAMI_INLINE void lbeq(void);
KONAMI_INLINE void lbge(void);
KONAMI_INLINE void lbgt(void);
KONAMI_INLINE void lbhi(void);
KONAMI_INLINE void lble(void);
KONAMI_INLINE void lbls(void);
KONAMI_INLINE void lblt(void);
KONAMI_INLINE void lbmi(void);
KONAMI_INLINE void lbne(void);
KONAMI_INLINE void lbpl(void);
KONAMI_INLINE void lbra(void);
KONAMI_INLINE void lbrn(void);
KONAMI_INLINE void lbsr(void);
KONAMI_INLINE void lbvc(void);
KONAMI_INLINE void lbvs(void);
KONAMI_INLINE void lda_di(void);
KONAMI_INLINE void lda_ex(void);
KONAMI_INLINE void lda_im(void);
KONAMI_INLINE void lda_ix(void);
KONAMI_INLINE void ldb_di(void);
KONAMI_INLINE void ldb_ex(void);
KONAMI_INLINE void ldb_im(void);
KONAMI_INLINE void ldb_ix(void);
KONAMI_INLINE void ldd_di(void);
KONAMI_INLINE void ldd_ex(void);
KONAMI_INLINE void ldd_im(void);
KONAMI_INLINE void ldd_ix(void);
KONAMI_INLINE void lds_di(void);
KONAMI_INLINE void lds_ex(void);
KONAMI_INLINE void lds_im(void);
KONAMI_INLINE void lds_ix(void);
KONAMI_INLINE void ldu_di(void);
KONAMI_INLINE void ldu_ex(void);
KONAMI_INLINE void ldu_im(void);
KONAMI_INLINE void ldu_ix(void);
KONAMI_INLINE void ldx_di(void);
KONAMI_INLINE void ldx_ex(void);
KONAMI_INLINE void ldx_im(void);
KONAMI_INLINE void ldx_ix(void);
KONAMI_INLINE void ldy_di(void);
KONAMI_INLINE void ldy_ex(void);
KONAMI_INLINE void ldy_im(void);
KONAMI_INLINE void ldy_ix(void);
KONAMI_INLINE void leas(void);
KONAMI_INLINE void leau(void);
KONAMI_INLINE void leax(void);
KONAMI_INLINE void leay(void);
KONAMI_INLINE void lsr_di(void);
KONAMI_INLINE void lsr_ex(void);
KONAMI_INLINE void lsr_ix(void);
KONAMI_INLINE void lsra(void);
KONAMI_INLINE void lsrb(void);
KONAMI_INLINE void mul(void);
KONAMI_INLINE void neg_di(void);
KONAMI_INLINE void neg_ex(void);
KONAMI_INLINE void neg_ix(void);
KONAMI_INLINE void nega(void);
KONAMI_INLINE void negb(void);
KONAMI_INLINE void nop(void);
KONAMI_INLINE void ora_di(void);
KONAMI_INLINE void ora_ex(void);
KONAMI_INLINE void ora_im(void);
KONAMI_INLINE void ora_ix(void);
KONAMI_INLINE void orb_di(void);
KONAMI_INLINE void orb_ex(void);
KONAMI_INLINE void orb_im(void);
KONAMI_INLINE void orb_ix(void);
KONAMI_INLINE void orcc(void);
KONAMI_INLINE void pshs(void);
KONAMI_INLINE void pshu(void);
KONAMI_INLINE void puls(void);
KONAMI_INLINE void pulu(void);
KONAMI_INLINE void rol_di(void);
KONAMI_INLINE void rol_ex(void);
KONAMI_INLINE void rol_ix(void);
KONAMI_INLINE void rola(void);
KONAMI_INLINE void rolb(void);
KONAMI_INLINE void ror_di(void);
KONAMI_INLINE void ror_ex(void);
KONAMI_INLINE void ror_ix(void);
KONAMI_INLINE void rora(void);
KONAMI_INLINE void rorb(void);
KONAMI_INLINE void rti(void);
KONAMI_INLINE void rts(void);
KONAMI_INLINE void sbca_di(void);
KONAMI_INLINE void sbca_ex(void);
KONAMI_INLINE void sbca_im(void);
KONAMI_INLINE void sbca_ix(void);
KONAMI_INLINE void sbcb_di(void);
KONAMI_INLINE void sbcb_ex(void);
KONAMI_INLINE void sbcb_im(void);
KONAMI_INLINE void sbcb_ix(void);
KONAMI_INLINE void sex(void);
KONAMI_INLINE void sta_di(void);
KONAMI_INLINE void sta_ex(void);
KONAMI_INLINE void sta_im(void);
KONAMI_INLINE void sta_ix(void);
KONAMI_INLINE void stb_di(void);
KONAMI_INLINE void stb_ex(void);
KONAMI_INLINE void stb_im(void);
KONAMI_INLINE void stb_ix(void);
KONAMI_INLINE void std_di(void);
KONAMI_INLINE void std_ex(void);
KONAMI_INLINE void std_im(void);
KONAMI_INLINE void std_ix(void);
KONAMI_INLINE void sts_di(void);
KONAMI_INLINE void sts_ex(void);
KONAMI_INLINE void sts_im(void);
KONAMI_INLINE void sts_ix(void);
KONAMI_INLINE void stu_di(void);
KONAMI_INLINE void stu_ex(void);
KONAMI_INLINE void stu_im(void);
KONAMI_INLINE void stu_ix(void);
KONAMI_INLINE void stx_di(void);
KONAMI_INLINE void stx_ex(void);
KONAMI_INLINE void stx_im(void);
KONAMI_INLINE void stx_ix(void);
KONAMI_INLINE void sty_di(void);
KONAMI_INLINE void sty_ex(void);
KONAMI_INLINE void sty_im(void);
KONAMI_INLINE void sty_ix(void);
KONAMI_INLINE void suba_di(void);
KONAMI_INLINE void suba_ex(void);
KONAMI_INLINE void suba_im(void);
KONAMI_INLINE void suba_ix(void);
KONAMI_INLINE void subb_di(void);
KONAMI_INLINE void subb_ex(void);
KONAMI_INLINE void subb_im(void);
KONAMI_INLINE void subb_ix(void);
KONAMI_INLINE void subd_di(void);
KONAMI_INLINE void subd_ex(void);
KONAMI_INLINE void subd_im(void);
KONAMI_INLINE void subd_ix(void);
KONAMI_INLINE void swi(void);
KONAMI_INLINE void swi2(void);
KONAMI_INLINE void swi3(void);
KONAMI_INLINE void sync(void);
KONAMI_INLINE void tfr(void);
KONAMI_INLINE void tst_di(void);
KONAMI_INLINE void tst_ex(void);
KONAMI_INLINE void tst_ix(void);
KONAMI_INLINE void tsta(void);
KONAMI_INLINE void tstb(void);

KONAMI_INLINE void clrd(void); /* 6309 */
KONAMI_INLINE void clrw_ix(void); /* 6309 ? */
KONAMI_INLINE void clrw_di(void); /* 6309 ? */
KONAMI_INLINE void clrw_ex(void); /* 6309 ? */
KONAMI_INLINE void negd(void);
KONAMI_INLINE void negw_ix(void); /* 6309 ? */
KONAMI_INLINE void negw_di(void); /* 6309 ? */
KONAMI_INLINE void negw_ex(void); /* 6309 ? */
KONAMI_INLINE void lsrd( void ); /* 6309 */
KONAMI_INLINE void lsrd_di( void ); /* 6309 */
KONAMI_INLINE void lsrd_ix( void ); /* 6309 */
KONAMI_INLINE void lsrd_ex( void ); /* 6309 */
KONAMI_INLINE void rord( void ); /* 6309 ? */
KONAMI_INLINE void rord_di( void ); /* 6309 */
KONAMI_INLINE void rord_ix( void ); /* 6309 */
KONAMI_INLINE void rord_ex( void ); /* 6309 */
KONAMI_INLINE void asrd( void ); /* 6309 ? */
KONAMI_INLINE void asrd_di( void ); /* 6309 */
KONAMI_INLINE void asrd_ix( void ); /* 6309 */
KONAMI_INLINE void asrd_ex( void ); /* 6309 */
KONAMI_INLINE void asld( void ); /* 6309 */
KONAMI_INLINE void asld_di( void ); /* 6309 */
KONAMI_INLINE void asld_ix( void ); /* 6309 */
KONAMI_INLINE void asld_ex( void ); /* 6309 */
KONAMI_INLINE void rold( void ); /* 6309 ? */
KONAMI_INLINE void rold_di( void ); /* 6309 */
KONAMI_INLINE void rold_ix( void ); /* 6309 */
KONAMI_INLINE void rold_ex( void ); /* 6309 */
KONAMI_INLINE void tstd(void);
KONAMI_INLINE void tstw_di( void );
KONAMI_INLINE void tstw_ix( void );
KONAMI_INLINE void tstw_ex( void );

/* Custom opcodes */
KONAMI_INLINE void setline_im( void );
KONAMI_INLINE void setline_ix( void );
KONAMI_INLINE void setline_di( void );
KONAMI_INLINE void setline_ex( void );
KONAMI_INLINE void bmove( void );
KONAMI_INLINE void move( void );
KONAMI_INLINE void decbjnz( void );
KONAMI_INLINE void decxjnz( void );
KONAMI_INLINE void bset( void );
KONAMI_INLINE void bset2( void );
KONAMI_INLINE void lmul(void);
KONAMI_INLINE void divx( void );
KONAMI_INLINE void incd( void );
KONAMI_INLINE void incw_di( void );
KONAMI_INLINE void incw_ix( void );
KONAMI_INLINE void incw_ex( void );
KONAMI_INLINE void decd( void );
KONAMI_INLINE void decw_di( void );
KONAMI_INLINE void decw_ix( void );
KONAMI_INLINE void decw_ex( void );
KONAMI_INLINE void lsrw_di( void );
KONAMI_INLINE void lsrw_ix( void );
KONAMI_INLINE void lsrw_ex( void );
KONAMI_INLINE void rorw_di( void );
KONAMI_INLINE void rorw_ix( void );
KONAMI_INLINE void rorw_ex( void );
KONAMI_INLINE void asrw_di( void );
KONAMI_INLINE void asrw_ix( void );
KONAMI_INLINE void asrw_ex( void );
KONAMI_INLINE void aslw_di( void );
KONAMI_INLINE void aslw_ix( void );
KONAMI_INLINE void aslw_ex( void );
KONAMI_INLINE void rolw_di( void );
KONAMI_INLINE void rolw_ix( void );
KONAMI_INLINE void rolw_ex( void );
KONAMI_INLINE void absa( void );
KONAMI_INLINE void absb( void );
KONAMI_INLINE void absd( void );

KONAMI_INLINE void opcode2( void );

static void (*const konami_main[0x100])(void) = {
	illegal,illegal,illegal,illegal,illegal,illegal,illegal,illegal,	/* 00 */
	opcode2,opcode2,opcode2,opcode2,pshs   ,pshu   ,puls   ,pulu   ,
	lda_im ,ldb_im ,opcode2,opcode2,adda_im,addb_im,opcode2,opcode2,	/* 10 */
	adca_im,adcb_im,opcode2,opcode2,suba_im,subb_im,opcode2,opcode2,
	sbca_im,sbcb_im,opcode2,opcode2,anda_im,andb_im,opcode2,opcode2,	/* 20 */
	bita_im,bitb_im,opcode2,opcode2,eora_im,eorb_im,opcode2,opcode2,
	ora_im ,orb_im ,opcode2,opcode2,cmpa_im,cmpb_im,opcode2,opcode2,	/* 30 */
	setline_im,opcode2,opcode2,opcode2,andcc,orcc  ,exg    ,tfr    ,
	ldd_im ,opcode2,ldx_im ,opcode2,ldy_im ,opcode2,ldu_im ,opcode2,	/* 40 */
	lds_im ,opcode2,cmpd_im,opcode2,cmpx_im,opcode2,cmpy_im,opcode2,
	cmpu_im,opcode2,cmps_im,opcode2,addd_im,opcode2,subd_im,opcode2,	/* 50 */
	opcode2,opcode2,opcode2,opcode2,opcode2,illegal,illegal,illegal,
	bra    ,bhi    ,bcc    ,bne    ,bvc    ,bpl    ,bge    ,bgt    ,	/* 60 */
	lbra   ,lbhi   ,lbcc   ,lbne   ,lbvc   ,lbpl   ,lbge   ,lbgt   ,
	brn    ,bls    ,bcs    ,beq    ,bvs    ,bmi    ,blt    ,ble    ,	/* 70 */
	lbrn   ,lbls   ,lbcs   ,lbeq   ,lbvs   ,lbmi   ,lblt   ,lble   ,
	clra   ,clrb   ,opcode2,coma   ,comb   ,opcode2,nega   ,negb   ,	/* 80 */
	opcode2,inca   ,incb   ,opcode2,deca   ,decb   ,opcode2,rts    ,
	tsta   ,tstb   ,opcode2,lsra   ,lsrb   ,opcode2,rora   ,rorb   ,	/* 90 */
	opcode2,asra   ,asrb   ,opcode2,asla   ,aslb   ,opcode2,rti    ,
	rola   ,rolb   ,opcode2,opcode2,opcode2,opcode2,opcode2,opcode2,	/* a0 */
	opcode2,opcode2,bsr    ,lbsr   ,decbjnz,decxjnz,nop    ,illegal,
	abx    ,daa	   ,sex    ,mul    ,lmul   ,divx   ,bmove  ,move   ,	/* b0 */
	lsrd   ,opcode2,rord   ,opcode2,asrd   ,opcode2,asld   ,opcode2,
	rold   ,opcode2,clrd   ,opcode2,negd   ,opcode2,incd   ,opcode2,	/* c0 */
	decd   ,opcode2,tstd   ,opcode2,absa   ,absb   ,absd   ,bset   ,
	bset2  ,illegal,illegal,illegal,illegal,illegal,illegal,illegal,	/* d0 */
	illegal,illegal,illegal,illegal,illegal,illegal,illegal,illegal,
	illegal,illegal,illegal,illegal,illegal,illegal,illegal,illegal,	/* e0 */
	illegal,illegal,illegal,illegal,illegal,illegal,illegal,illegal,
	illegal,illegal,illegal,illegal,illegal,illegal,illegal,illegal,	/* f0 */
	illegal,illegal,illegal,illegal,illegal,illegal,illegal,illegal
};

static void (*const konami_indexed[0x100])(void) = {
	illegal,illegal,illegal,illegal,illegal,illegal,illegal,illegal,	/* 00 */
	leax   ,leay   ,leau   ,leas   ,illegal,illegal,illegal,illegal,
	illegal,illegal,lda_ix ,ldb_ix ,illegal,illegal,adda_ix,addb_ix,	/* 10 */
	illegal,illegal,adca_ix,adcb_ix,illegal,illegal,suba_ix,subb_ix,
	illegal,illegal,sbca_ix,sbcb_ix,illegal,illegal,anda_ix,andb_ix,	/* 20 */
	illegal,illegal,bita_ix,bitb_ix,illegal,illegal,eora_ix,eorb_ix,
	illegal,illegal,ora_ix ,orb_ix ,illegal,illegal,cmpa_ix,cmpb_ix,	/* 30 */
	illegal,setline_ix,sta_ix,stb_ix,illegal,illegal,illegal,illegal,
	illegal,ldd_ix ,illegal,ldx_ix ,illegal,ldy_ix ,illegal,ldu_ix ,	/* 40 */
	illegal,lds_ix ,illegal,cmpd_ix,illegal,cmpx_ix,illegal,cmpy_ix,
	illegal,cmpu_ix,illegal,cmps_ix,illegal,addd_ix,illegal,subd_ix,	/* 50 */
	std_ix ,stx_ix ,sty_ix ,stu_ix ,sts_ix ,illegal,illegal,illegal,
	illegal,illegal,illegal,illegal,illegal,illegal,illegal,illegal,	/* 60 */
	illegal,illegal,illegal,illegal,illegal,illegal,illegal,illegal,
	illegal,illegal,illegal,illegal,illegal,illegal,illegal,illegal,	/* 70 */
	illegal,illegal,illegal,illegal,illegal,illegal,illegal,illegal,
	illegal,illegal,clr_ix ,illegal,illegal,com_ix ,illegal,illegal,	/* 80 */
	neg_ix ,illegal,illegal,inc_ix ,illegal,illegal,dec_ix ,illegal,
	illegal,illegal,tst_ix ,illegal,illegal,lsr_ix ,illegal,illegal,	/* 90 */
	ror_ix ,illegal,illegal,asr_ix ,illegal,illegal,asl_ix ,illegal,
	illegal,illegal,rol_ix ,lsrw_ix,rorw_ix,asrw_ix,aslw_ix,rolw_ix,	/* a0 */
	jmp_ix ,jsr_ix ,illegal,illegal,illegal,illegal,illegal,illegal,
	illegal,illegal,illegal,illegal,illegal,illegal,illegal,illegal,	/* b0 */
	illegal,lsrd_ix,illegal,rord_ix,illegal,asrd_ix,illegal,asld_ix,
	illegal,rold_ix,illegal,clrw_ix,illegal,negw_ix,illegal,incw_ix,	/* c0 */
	illegal,decw_ix,illegal,tstw_ix,illegal,illegal,illegal,illegal,
	illegal,illegal,illegal,illegal,illegal,illegal,illegal,illegal,	/* d0 */
	illegal,illegal,illegal,illegal,illegal,illegal,illegal,illegal,
	illegal,illegal,illegal,illegal,illegal,illegal,illegal,illegal,	/* e0 */
	illegal,illegal,illegal,illegal,illegal,illegal,illegal,illegal,
	illegal,illegal,illegal,illegal,illegal,illegal,illegal,illegal,	/* f0 */
	illegal,illegal,illegal,illegal,illegal,illegal,illegal,illegal
};

static void (*const konami_direct[0x100])(void) = {
	illegal,illegal,illegal,illegal,illegal,illegal,illegal,illegal,	/* 00 */
	illegal,illegal,illegal,illegal,illegal,illegal,illegal,illegal,
	illegal,illegal,lda_di ,ldb_di ,illegal,illegal,adda_di,addb_di,	/* 10 */
	illegal,illegal,adca_di,adcb_di,illegal,illegal,suba_di,subb_di,
	illegal,illegal,sbca_di,sbcb_di,illegal,illegal,anda_di,andb_di,	/* 20 */
	illegal,illegal,bita_di,bitb_di,illegal,illegal,eora_di,eorb_di,
	illegal,illegal,ora_di ,orb_di ,illegal,illegal,cmpa_di,cmpb_di,	/* 30 */
	illegal,setline_di,sta_di,stb_di,illegal,illegal,illegal,illegal,
	illegal,ldd_di ,illegal,ldx_di ,illegal,ldy_di ,illegal,ldu_di ,	/* 40 */
	illegal,lds_di ,illegal,cmpd_di,illegal,cmpx_di,illegal,cmpy_di,
	illegal,cmpu_di,illegal,cmps_di,illegal,addd_di,illegal,subd_di,	/* 50 */
	std_di ,stx_di ,sty_di ,stu_di ,sts_di ,illegal,illegal,illegal,
	illegal,illegal,illegal,illegal,illegal,illegal,illegal,illegal,	/* 60 */
	illegal,illegal,illegal,illegal,illegal,illegal,illegal,illegal,
	illegal,illegal,illegal,illegal,illegal,illegal,illegal,illegal,	/* 70 */
	illegal,illegal,illegal,illegal,illegal,illegal,illegal,illegal,
	illegal,illegal,clr_di ,illegal,illegal,com_di ,illegal,illegal,	/* 80 */
	neg_di ,illegal,illegal,inc_di ,illegal,illegal,dec_di ,illegal,
	illegal,illegal,tst_di ,illegal,illegal,lsr_di ,illegal,illegal,	/* 90 */
	ror_di ,illegal,illegal,asr_di ,illegal,illegal,asl_di ,illegal,
	illegal,illegal,rol_di ,lsrw_di,rorw_di,asrw_di,aslw_di,rolw_di,	/* a0 */
	jmp_di ,jsr_di ,illegal,illegal,illegal,illegal,illegal,illegal,
	illegal,illegal,illegal,illegal,illegal,illegal,illegal,illegal,	/* b0 */
	illegal,lsrd_di,illegal,rord_di,illegal,asrd_di,illegal,asld_di,
	illegal,rold_di,illegal,clrw_di,illegal,negw_di,illegal,incw_di,	/* c0 */
	illegal,decw_di,illegal,tstw_di,illegal,illegal,illegal,illegal,
	illegal,illegal,illegal,illegal,illegal,illegal,illegal,illegal,	/* d0 */
	illegal,illegal,illegal,illegal,illegal,illegal,illegal,illegal,
	illegal,illegal,illegal,illegal,illegal,illegal,illegal,illegal,	/* e0 */
	illegal,illegal,illegal,illegal,illegal,illegal,illegal,illegal,
	illegal,illegal,illegal,illegal,illegal,illegal,illegal,illegal,	/* f0 */
	illegal,illegal,illegal,illegal,illegal,illegal,illegal,illegal
};

static void (*const konami_extended[0x100])(void) = {
	illegal,illegal,illegal,illegal,illegal,illegal,illegal,illegal,	/* 00 */
	illegal,illegal,illegal,illegal,illegal,illegal,illegal,illegal,
	illegal,illegal,lda_ex ,ldb_ex ,illegal,illegal,adda_ex,addb_ex,	/* 10 */
	illegal,illegal,adca_ex,adcb_ex,illegal,illegal,suba_ex,subb_ex,
	illegal,illegal,sbca_ex,sbcb_ex,illegal,illegal,anda_ex,andb_ex,	/* 20 */
	illegal,illegal,bita_ex,bitb_ex,illegal,illegal,eora_ex,eorb_ex,
	illegal,illegal,ora_ex ,orb_ex ,illegal,illegal,cmpa_ex,cmpb_ex,	/* 30 */
	illegal,setline_ex,sta_ex,stb_ex,illegal,illegal,illegal,illegal,
	illegal,ldd_ex ,illegal,ldx_ex ,illegal,ldy_ex ,illegal,ldu_ex ,	/* 40 */
	illegal,lds_ex ,illegal,cmpd_ex,illegal,cmpx_ex,illegal,cmpy_ex,
	illegal,cmpu_ex,illegal,cmps_ex,illegal,addd_ex,illegal,subd_ex,	/* 50 */
	std_ex ,stx_ex ,sty_ex ,stu_ex ,sts_ex ,illegal,illegal,illegal,
	illegal,illegal,illegal,illegal,illegal,illegal,illegal,illegal,	/* 60 */
	illegal,illegal,illegal,illegal,illegal,illegal,illegal,illegal,
	illegal,illegal,illegal,illegal,illegal,illegal,illegal,illegal,	/* 70 */
	illegal,illegal,illegal,illegal,illegal,illegal,illegal,illegal,
	illegal,illegal,clr_ex ,illegal,illegal,com_ex ,illegal,illegal,	/* 80 */
	neg_ex ,illegal,illegal,inc_ex ,illegal,illegal,dec_ex ,illegal,
	illegal,illegal,tst_ex ,illegal,illegal,lsr_ex ,illegal,illegal,	/* 90 */
	ror_ex ,illegal,illegal,asr_ex ,illegal,illegal,asl_ex ,illegal,
	illegal,illegal,rol_ex ,lsrw_ex,rorw_ex,asrw_ex,aslw_ex,rolw_ex,	/* a0 */
	jmp_ex ,jsr_ex ,illegal,illegal,illegal,illegal,illegal,illegal,
	illegal,illegal,illegal,illegal,illegal,illegal,illegal,illegal,	/* b0 */
	illegal,lsrd_ex,illegal,rord_ex,illegal,asrd_ex,illegal,asld_ex,
	illegal,rold_ex,illegal,clrw_ex,illegal,negw_ex,illegal,incw_ex,	/* c0 */
	illegal,decw_ex,illegal,tstw_ex,illegal,illegal,illegal,illegal,
	illegal,illegal,illegal,illegal,illegal,illegal,illegal,illegal,	/* d0 */
	illegal,illegal,illegal,illegal,illegal,illegal,illegal,illegal,
	illegal,illegal,illegal,illegal,illegal,illegal,illegal,illegal,	/* e0 */
	illegal,illegal,illegal,illegal,illegal,illegal,illegal,illegal,
	illegal,illegal,illegal,illegal,illegal,illegal,illegal,illegal,	/* f0 */
	illegal,illegal,illegal,illegal,illegal,illegal,illegal,illegal
};

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
KONAMI_INLINE void illegal( void )
#endif
{
	logerror("KONAMI: illegal opcode at %04x\n",PC);
}

/* $00 NEG direct ?**** */
KONAMI_INLINE void neg_di( void )
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
KONAMI_INLINE void com_di( void )
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
KONAMI_INLINE void lsr_di( void )
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
KONAMI_INLINE void ror_di( void )
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
KONAMI_INLINE void asr_di( void )
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
KONAMI_INLINE void asl_di( void )
{
	UINT16 t,r;
	DIRBYTE(t);
	r = t << 1;
	CLR_NZVC;
	SET_FLAGS8(t,t,r);
	WM(EAD,r);
}

/* $09 ROL direct -**** */
KONAMI_INLINE void rol_di( void )
{
	UINT16 t,r;
	DIRBYTE(t);
	r = (CC & CC_C) | (t << 1);
	CLR_NZVC;
	SET_FLAGS8(t,t,r);
	WM(EAD,r);
}

/* $0A DEC direct -***- */
KONAMI_INLINE void dec_di( void )
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
KONAMI_INLINE void inc_di( void )
{
	UINT8 t;
	DIRBYTE(t);
	++t;
	CLR_NZV;
	SET_FLAGS8I(t);
	WM(EAD,t);
}

/* $OD TST direct -**0- */
KONAMI_INLINE void tst_di( void )
{
	UINT8 t;
	DIRBYTE(t);
	CLR_NZV;
	SET_NZ8(t);
}

/* $0E JMP direct ----- */
KONAMI_INLINE void jmp_di( void )
{
    DIRECT;
	PCD=EAD;
	change_pc(PCD);
}

/* $0F CLR direct -0100 */
KONAMI_INLINE void clr_di( void )
{
	DIRECT;
	WM(EAD,0);
	CLR_NZVC;
	SEZ;
}

/* $10 FLAG */

/* $11 FLAG */

/* $12 NOP inherent ----- */
KONAMI_INLINE void nop( void )
{
	;
}

/* $13 SYNC inherent ----- */
KONAMI_INLINE void sync( void )
{
	/* SYNC stops processing instructions until an interrupt request happens. */
	/* This doesn't require the corresponding interrupt to be enabled: if it */
	/* is disabled, execution continues with the next instruction. */
	konami.int_state |= KONAMI_SYNC;
	CHECK_IRQ_LINES;
	/* if KONAMI_SYNC has not been cleared by CHECK_IRQ_LINES,
     * stop execution until the interrupt lines change. */
	if( (konami.int_state & KONAMI_SYNC) && konami_ICount > 0 )
		konami_ICount = 0;
}

/* $14 ILLEGAL */

/* $15 ILLEGAL */

/* $16 LBRA relative ----- */
KONAMI_INLINE void lbra( void )
{
	IMMWORD(ea);
	PC += EA;
	change_pc(PCD);

	/* EHC 980508 speed up busy loop */
	if( EA == 0xfffd && konami_ICount > 0 )
		konami_ICount = 0;
}

/* $17 LBSR relative ----- */
KONAMI_INLINE void lbsr( void )
{
	IMMWORD(ea);
	PUSHWORD(pPC);
	PC += EA;
	change_pc(PCD);
}

/* $18 ILLEGAL */

#if 1
/* $19 DAA inherent (A) -**0* */
KONAMI_INLINE void daa( void )
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
#else
/* $19 DAA inherent (A) -**0* */
KONAMI_INLINE void daa( void )
{
	UINT16 t;
	t = A;
	if (CC & CC_H) t+=0x06;
	if ((t&0x0f)>9) t+=0x06;		/* ASG -- this code is broken! $66+$99=$FF -> DAA should = $65, we get $05! */
	if (CC & CC_C) t+=0x60;
	if ((t&0xf0)>0x90) t+=0x60;
	if (t&0x100) SEC;
	A = t;
}
#endif

/* $1A ORCC immediate ##### */
KONAMI_INLINE void orcc( void )
{
	UINT8 t;
	IMMBYTE(t);
	CC |= t;
	CHECK_IRQ_LINES;
}

/* $1B ILLEGAL */

/* $1C ANDCC immediate ##### */
KONAMI_INLINE void andcc( void )
{
	UINT8 t;
	IMMBYTE(t);
	CC &= t;
	CHECK_IRQ_LINES;
}

/* $1D SEX inherent -**0- */
KONAMI_INLINE void sex( void )
{
	UINT16 t;
	t = SIGNED(B);
	D = t;
//  CLR_NZV;    NS 20020905: applying the same fix that was applied to 6809 and 6309
	CLR_NZ;
	SET_NZ16(t);
}

/* $1E EXG inherent ----- */
KONAMI_INLINE void exg( void )
{
	UINT16 t1 = 0, t2 = 0;
	UINT8 tb;

	IMMBYTE(tb);

	GETREG( t1, tb >> 4 );
	GETREG( t2, tb & 0x0f );

	SETREG( t2, tb >> 4 );
	SETREG( t1, tb & 0x0f );
}

/* $1F TFR inherent ----- */
KONAMI_INLINE void tfr( void )
{
	UINT8 tb;
	UINT16 t = 0;

	IMMBYTE(tb);

	GETREG( t, tb & 0x0f );
	SETREG( t, ( tb >> 4 ) & 0x07 );
}

/* $20 BRA relative ----- */
KONAMI_INLINE void bra( void )
{
	UINT8 t;
	IMMBYTE(t);
	PC += SIGNED(t);
	change_pc(PCD);
	/* JB 970823 - speed up busy loops */
	if( t == 0xfe && konami_ICount > 0 )
		konami_ICount = 0;
}

/* $21 BRN relative ----- */
KONAMI_INLINE void brn( void )
{
	UINT8 t;
	IMMBYTE(t);
}

/* $1021 LBRN relative ----- */
KONAMI_INLINE void lbrn( void )
{
	IMMWORD(ea);
}

/* $22 BHI relative ----- */
KONAMI_INLINE void bhi( void )
{
	BRANCH( !(CC & (CC_Z|CC_C)) );
}

/* $1022 LBHI relative ----- */
KONAMI_INLINE void lbhi( void )
{
	LBRANCH( !(CC & (CC_Z|CC_C)) );
}

/* $23 BLS relative ----- */
KONAMI_INLINE void bls( void )
{
	BRANCH( (CC & (CC_Z|CC_C)) );
}

/* $1023 LBLS relative ----- */
KONAMI_INLINE void lbls( void )
{
	LBRANCH( (CC&(CC_Z|CC_C)) );
}

/* $24 BCC relative ----- */
KONAMI_INLINE void bcc( void )
{
	BRANCH( !(CC&CC_C) );
}

/* $1024 LBCC relative ----- */
KONAMI_INLINE void lbcc( void )
{
	LBRANCH( !(CC&CC_C) );
}

/* $25 BCS relative ----- */
KONAMI_INLINE void bcs( void )
{
	BRANCH( (CC&CC_C) );
}

/* $1025 LBCS relative ----- */
KONAMI_INLINE void lbcs( void )
{
	LBRANCH( (CC&CC_C) );
}

/* $26 BNE relative ----- */
KONAMI_INLINE void bne( void )
{
	BRANCH( !(CC&CC_Z) );
}

/* $1026 LBNE relative ----- */
KONAMI_INLINE void lbne( void )
{
	LBRANCH( !(CC&CC_Z) );
}

/* $27 BEQ relative ----- */
KONAMI_INLINE void beq( void )
{
	BRANCH( (CC&CC_Z) );
}

/* $1027 LBEQ relative ----- */
KONAMI_INLINE void lbeq( void )
{
	LBRANCH( (CC&CC_Z) );
}

/* $28 BVC relative ----- */
KONAMI_INLINE void bvc( void )
{
	BRANCH( !(CC&CC_V) );
}

/* $1028 LBVC relative ----- */
KONAMI_INLINE void lbvc( void )
{
	LBRANCH( !(CC&CC_V) );
}

/* $29 BVS relative ----- */
KONAMI_INLINE void bvs( void )
{
	BRANCH( (CC&CC_V) );
}

/* $1029 LBVS relative ----- */
KONAMI_INLINE void lbvs( void )
{
	LBRANCH( (CC&CC_V) );
}

/* $2A BPL relative ----- */
KONAMI_INLINE void bpl( void )
{
	BRANCH( !(CC&CC_N) );
}

/* $102A LBPL relative ----- */
KONAMI_INLINE void lbpl( void )
{
	LBRANCH( !(CC&CC_N) );
}

/* $2B BMI relative ----- */
KONAMI_INLINE void bmi( void )
{
	BRANCH( (CC&CC_N) );
}

/* $102B LBMI relative ----- */
KONAMI_INLINE void lbmi( void )
{
	LBRANCH( (CC&CC_N) );
}

/* $2C BGE relative ----- */
KONAMI_INLINE void bge( void )
{
	BRANCH( !NXORV );
}

/* $102C LBGE relative ----- */
KONAMI_INLINE void lbge( void )
{
	LBRANCH( !NXORV );
}

/* $2D BLT relative ----- */
KONAMI_INLINE void blt( void )
{
	BRANCH( NXORV );
}

/* $102D LBLT relative ----- */
KONAMI_INLINE void lblt( void )
{
	LBRANCH( NXORV );
}

/* $2E BGT relative ----- */
KONAMI_INLINE void bgt( void )
{
	BRANCH( !(NXORV || (CC&CC_Z)) );
}

/* $102E LBGT relative ----- */
KONAMI_INLINE void lbgt( void )
{
	LBRANCH( !(NXORV || (CC&CC_Z)) );
}

/* $2F BLE relative ----- */
KONAMI_INLINE void ble( void )
{
	BRANCH( (NXORV || (CC&CC_Z)) );
}

/* $102F LBLE relative ----- */
KONAMI_INLINE void lble( void )
{
	LBRANCH( (NXORV || (CC&CC_Z)) );
}

/* $30 LEAX indexed --*-- */
KONAMI_INLINE void leax( void )
{
	X = EA;
	CLR_Z;
	SET_Z(X);
}

/* $31 LEAY indexed --*-- */
KONAMI_INLINE void leay( void )
{
	Y = EA;
	CLR_Z;
	SET_Z(Y);
}

/* $32 LEAS indexed ----- */
KONAMI_INLINE void leas( void )
{
	S = EA;
	konami.int_state |= KONAMI_LDS;
}

/* $33 LEAU indexed ----- */
KONAMI_INLINE void leau( void )
{
	U = EA;
}

/* $34 PSHS inherent ----- */
KONAMI_INLINE void pshs( void )
{
	UINT8 t;
	IMMBYTE(t);
	if( t&0x80 ) { PUSHWORD(pPC); konami_ICount -= 2; }
	if( t&0x40 ) { PUSHWORD(pU);  konami_ICount -= 2; }
	if( t&0x20 ) { PUSHWORD(pY);  konami_ICount -= 2; }
	if( t&0x10 ) { PUSHWORD(pX);  konami_ICount -= 2; }
	if( t&0x08 ) { PUSHBYTE(DP);  konami_ICount -= 1; }
	if( t&0x04 ) { PUSHBYTE(B);   konami_ICount -= 1; }
	if( t&0x02 ) { PUSHBYTE(A);   konami_ICount -= 1; }
	if( t&0x01 ) { PUSHBYTE(CC);  konami_ICount -= 1; }
}

/* 35 PULS inherent ----- */
KONAMI_INLINE void puls( void )
{
	UINT8 t;
	IMMBYTE(t);
	if( t&0x01 ) { PULLBYTE(CC); konami_ICount -= 1; }
	if( t&0x02 ) { PULLBYTE(A);  konami_ICount -= 1; }
	if( t&0x04 ) { PULLBYTE(B);  konami_ICount -= 1; }
	if( t&0x08 ) { PULLBYTE(DP); konami_ICount -= 1; }
	if( t&0x10 ) { PULLWORD(XD); konami_ICount -= 2; }
	if( t&0x20 ) { PULLWORD(YD); konami_ICount -= 2; }
	if( t&0x40 ) { PULLWORD(UD); konami_ICount -= 2; }
	if( t&0x80 ) { PULLWORD(PCD); change_pc(PCD); konami_ICount -= 2; }

	/* check after all PULLs */
	if( t&0x01 ) { CHECK_IRQ_LINES; }
}

/* $36 PSHU inherent ----- */
KONAMI_INLINE void pshu( void )
{
	UINT8 t;
	IMMBYTE(t);
	if( t&0x80 ) { PSHUWORD(pPC); konami_ICount -= 2; }
	if( t&0x40 ) { PSHUWORD(pS);  konami_ICount -= 2; }
	if( t&0x20 ) { PSHUWORD(pY);  konami_ICount -= 2; }
	if( t&0x10 ) { PSHUWORD(pX);  konami_ICount -= 2; }
	if( t&0x08 ) { PSHUBYTE(DP);  konami_ICount -= 1; }
	if( t&0x04 ) { PSHUBYTE(B);   konami_ICount -= 1; }
	if( t&0x02 ) { PSHUBYTE(A);   konami_ICount -= 1; }
	if( t&0x01 ) { PSHUBYTE(CC);  konami_ICount -= 1; }
}

/* 37 PULU inherent ----- */
KONAMI_INLINE void pulu( void )
{
	UINT8 t;
	IMMBYTE(t);
	if( t&0x01 ) { PULUBYTE(CC); konami_ICount -= 1; }
	if( t&0x02 ) { PULUBYTE(A);  konami_ICount -= 1; }
	if( t&0x04 ) { PULUBYTE(B);  konami_ICount -= 1; }
	if( t&0x08 ) { PULUBYTE(DP); konami_ICount -= 1; }
	if( t&0x10 ) { PULUWORD(XD); konami_ICount -= 2; }
	if( t&0x20 ) { PULUWORD(YD); konami_ICount -= 2; }
	if( t&0x40 ) { PULUWORD(SD); konami_ICount -= 2; }
	if( t&0x80 ) { PULUWORD(PCD); change_pc(PCD); konami_ICount -= 2; }

	/* check after all PULLs */
	if( t&0x01 ) { CHECK_IRQ_LINES; }
}

/* $38 ILLEGAL */

/* $39 RTS inherent ----- */
KONAMI_INLINE void rts( void )
{
	PULLWORD(PCD);
	change_pc(PCD);
}

/* $3A ABX inherent ----- */
KONAMI_INLINE void abx( void )
{
	X += B;
}

/* $3B RTI inherent ##### */
KONAMI_INLINE void rti( void )
{
	PULLBYTE(CC);
	if( CC & CC_E ) /* entire state saved? */
	{
        konami_ICount -= 9;
		PULLBYTE(A);
		PULLBYTE(B);
		PULLBYTE(DP);
		PULLWORD(XD);
		PULLWORD(YD);
		PULLWORD(UD);
	}
	PULLWORD(PCD);
	change_pc(PCD);
	CHECK_IRQ_LINES;
}

/* $3C CWAI inherent ----1 */
KONAMI_INLINE void cwai( void )
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
	konami.int_state |= KONAMI_CWAI;
	CHECK_IRQ_LINES;
	if( (konami.int_state & KONAMI_CWAI) && konami_ICount > 0 )
		konami_ICount = 0;
}

/* $3D MUL inherent --*-@ */
KONAMI_INLINE void mul( void )
{
	UINT16 t;
	t = A * B;
	CLR_ZC; SET_Z16(t); if(t&0x80) SEC;
	D = t;
}

/* $3E ILLEGAL */

/* $3F SWI (SWI2 SWI3) absolute indirect ----- */
KONAMI_INLINE void swi( void )
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
	change_pc(PCD);
}

/* $103F SWI2 absolute indirect ----- */
KONAMI_INLINE void swi2( void )
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
	PCD=RM16(0xfff4);
	change_pc(PCD);
}

/* $113F SWI3 absolute indirect ----- */
KONAMI_INLINE void swi3( void )
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
	PCD=RM16(0xfff2);
	change_pc(PCD);
}

/* $40 NEGA inherent ?**** */
KONAMI_INLINE void nega( void )
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
KONAMI_INLINE void coma( void )
{
	A = ~A;
	CLR_NZV;
	SET_NZ8(A);
	SEC;
}

/* $44 LSRA inherent -0*-* */
KONAMI_INLINE void lsra( void )
{
	CLR_NZC;
	CC |= (A & CC_C);
	A >>= 1;
	SET_Z8(A);
}

/* $45 ILLEGAL */

/* $46 RORA inherent -**-* */
KONAMI_INLINE void rora( void )
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
KONAMI_INLINE void asra( void )
{
	CLR_NZC;
	CC |= (A & CC_C);
	A = (A & 0x80) | (A >> 1);
	SET_NZ8(A);
}

/* $48 ASLA inherent ?**** */
KONAMI_INLINE void asla( void )
{
	UINT16 r;
	r = A << 1;
	CLR_NZVC;
	SET_FLAGS8(A,A,r);
	A = r;
}

/* $49 ROLA inherent -**** */
KONAMI_INLINE void rola( void )
{
	UINT16 t,r;
	t = A;
	r = (CC & CC_C) | (t<<1);
	CLR_NZVC; SET_FLAGS8(t,t,r);
	A = r;
}

/* $4A DECA inherent -***- */
KONAMI_INLINE void deca( void )
{
	--A;
	CLR_NZV;
	SET_FLAGS8D(A);
}

/* $4B ILLEGAL */

/* $4C INCA inherent -***- */
KONAMI_INLINE void inca( void )
{
	++A;
	CLR_NZV;
	SET_FLAGS8I(A);
}

/* $4D TSTA inherent -**0- */
KONAMI_INLINE void tsta( void )
{
	CLR_NZV;
	SET_NZ8(A);
}

/* $4E ILLEGAL */

/* $4F CLRA inherent -0100 */
KONAMI_INLINE void clra( void )
{
	A = 0;
	CLR_NZVC; SEZ;
}

/* $50 NEGB inherent ?**** */
KONAMI_INLINE void negb( void )
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
KONAMI_INLINE void comb( void )
{
	B = ~B;
	CLR_NZV;
	SET_NZ8(B);
	SEC;
}

/* $54 LSRB inherent -0*-* */
KONAMI_INLINE void lsrb( void )
{
	CLR_NZC;
	CC |= (B & CC_C);
	B >>= 1;
	SET_Z8(B);
}

/* $55 ILLEGAL */

/* $56 RORB inherent -**-* */
KONAMI_INLINE void rorb( void )
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
KONAMI_INLINE void asrb( void )
{
	CLR_NZC;
	CC |= (B & CC_C);
	B= (B & 0x80) | (B >> 1);
	SET_NZ8(B);
}

/* $58 ASLB inherent ?**** */
KONAMI_INLINE void aslb( void )
{
	UINT16 r;
	r = B << 1;
	CLR_NZVC;
	SET_FLAGS8(B,B,r);
	B = r;
}

/* $59 ROLB inherent -**** */
KONAMI_INLINE void rolb( void )
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
KONAMI_INLINE void decb( void )
{
	--B;
	CLR_NZV;
	SET_FLAGS8D(B);
}

/* $5B ILLEGAL */

/* $5C INCB inherent -***- */
KONAMI_INLINE void incb( void )
{
	++B;
	CLR_NZV;
	SET_FLAGS8I(B);
}

/* $5D TSTB inherent -**0- */
KONAMI_INLINE void tstb( void )
{
	CLR_NZV;
	SET_NZ8(B);
}

/* $5E ILLEGAL */

/* $5F CLRB inherent -0100 */
KONAMI_INLINE void clrb( void )
{
	B = 0;
	CLR_NZVC; SEZ;
}

/* $60 NEG indexed ?**** */
KONAMI_INLINE void neg_ix( void )
{
	UINT16 r,t;
	t = RM(EAD);
	r = -t;
	CLR_NZVC;
	SET_FLAGS8(0,t,r);
	WM(EAD,r);
}

/* $61 ILLEGAL */

/* $62 ILLEGAL */

/* $63 COM indexed -**01 */
KONAMI_INLINE void com_ix( void )
{
	UINT8 t;
	t = ~RM(EAD);
	CLR_NZV;
	SET_NZ8(t);
	SEC;
	WM(EAD,t);
}

/* $64 LSR indexed -0*-* */
KONAMI_INLINE void lsr_ix( void )
{
	UINT8 t;
	t = RM(EAD);
	CLR_NZC;
	CC |= (t & CC_C);
	t>>=1; SET_Z8(t);
	WM(EAD,t);
}

/* $65 ILLEGAL */

/* $66 ROR indexed -**-* */
KONAMI_INLINE void ror_ix( void )
{
	UINT8 t,r;
	t = RM(EAD);
	r = (CC & CC_C) << 7;
	CLR_NZC;
	CC |= (t & CC_C);
	r |= t>>1; SET_NZ8(r);
	WM(EAD,r);
}

/* $67 ASR indexed ?**-* */
KONAMI_INLINE void asr_ix( void )
{
	UINT8 t;
	t = RM(EAD);
	CLR_NZC;
	CC |= (t & CC_C);
	t=(t&0x80)|(t>>1);
	SET_NZ8(t);
	WM(EAD,t);
}

/* $68 ASL indexed ?**** */
KONAMI_INLINE void asl_ix( void )
{
	UINT16 t,r;
	t = RM(EAD);
	r = t << 1;
	CLR_NZVC;
	SET_FLAGS8(t,t,r);
	WM(EAD,r);
}

/* $69 ROL indexed -**** */
KONAMI_INLINE void rol_ix( void )
{
	UINT16 t,r;
	t = RM(EAD);
	r = CC & CC_C;
	r |= t << 1;
	CLR_NZVC;
	SET_FLAGS8(t,t,r);
	WM(EAD,r);
}

/* $6A DEC indexed -***- */
KONAMI_INLINE void dec_ix( void )
{
	UINT8 t;
	t = RM(EAD) - 1;
	CLR_NZV; SET_FLAGS8D(t);
	WM(EAD,t);
}

/* $6B ILLEGAL */

/* $6C INC indexed -***- */
KONAMI_INLINE void inc_ix( void )
{
	UINT8 t;
	t = RM(EAD) + 1;
	CLR_NZV; SET_FLAGS8I(t);
	WM(EAD,t);
}

/* $6D TST indexed -**0- */
KONAMI_INLINE void tst_ix( void )
{
	UINT8 t;
	t = RM(EAD);
	CLR_NZV;
	SET_NZ8(t);
}

/* $6E JMP indexed ----- */
KONAMI_INLINE void jmp_ix( void )
{
	PCD=EAD;
	change_pc(PCD);
}

/* $6F CLR indexed -0100 */
KONAMI_INLINE void clr_ix( void )
{
	WM(EAD,0);
	CLR_NZVC; SEZ;
}

/* $70 NEG extended ?**** */
KONAMI_INLINE void neg_ex( void )
{
	UINT16 r,t;
	EXTBYTE(t); r=-t;
	CLR_NZVC; SET_FLAGS8(0,t,r);
	WM(EAD,r);
}

/* $71 ILLEGAL */

/* $72 ILLEGAL */

/* $73 COM extended -**01 */
KONAMI_INLINE void com_ex( void )
{
	UINT8 t;
	EXTBYTE(t); t = ~t;
	CLR_NZV; SET_NZ8(t); SEC;
	WM(EAD,t);
}

/* $74 LSR extended -0*-* */
KONAMI_INLINE void lsr_ex( void )
{
	UINT8 t;
	EXTBYTE(t); CLR_NZC; CC |= (t & CC_C);
	t>>=1; SET_Z8(t);
	WM(EAD,t);
}

/* $75 ILLEGAL */

/* $76 ROR extended -**-* */
KONAMI_INLINE void ror_ex( void )
{
	UINT8 t,r;
	EXTBYTE(t); r=(CC & CC_C) << 7;
	CLR_NZC; CC |= (t & CC_C);
	r |= t>>1; SET_NZ8(r);
	WM(EAD,r);
}

/* $77 ASR extended ?**-* */
KONAMI_INLINE void asr_ex( void )
{
	UINT8 t;
	EXTBYTE(t); CLR_NZC; CC |= (t & CC_C);
	t=(t&0x80)|(t>>1);
	SET_NZ8(t);
	WM(EAD,t);
}

/* $78 ASL extended ?**** */
KONAMI_INLINE void asl_ex( void )
{
	UINT16 t,r;
	EXTBYTE(t); r=t<<1;
	CLR_NZVC; SET_FLAGS8(t,t,r);
	WM(EAD,r);
}

/* $79 ROL extended -**** */
KONAMI_INLINE void rol_ex( void )
{
	UINT16 t,r;
	EXTBYTE(t); r = (CC & CC_C) | (t << 1);
	CLR_NZVC; SET_FLAGS8(t,t,r);
	WM(EAD,r);
}

/* $7A DEC extended -***- */
KONAMI_INLINE void dec_ex( void )
{
	UINT8 t;
	EXTBYTE(t); --t;
	CLR_NZV; SET_FLAGS8D(t);
	WM(EAD,t);
}

/* $7B ILLEGAL */

/* $7C INC extended -***- */
KONAMI_INLINE void inc_ex( void )
{
	UINT8 t;
	EXTBYTE(t); ++t;
	CLR_NZV; SET_FLAGS8I(t);
	WM(EAD,t);
}

/* $7D TST extended -**0- */
KONAMI_INLINE void tst_ex( void )
{
	UINT8 t;
	EXTBYTE(t); CLR_NZV; SET_NZ8(t);
}

/* $7E JMP extended ----- */
KONAMI_INLINE void jmp_ex( void )
{
	EXTENDED;
	PCD=EAD;
	change_pc(PCD);
}

/* $7F CLR extended -0100 */
KONAMI_INLINE void clr_ex( void )
{
	EXTENDED;
	WM(EAD,0);
	CLR_NZVC; SEZ;
}

/* $80 SUBA immediate ?**** */
KONAMI_INLINE void suba_im( void )
{
	UINT16 t,r;
	IMMBYTE(t);
	r = A - t;
	CLR_NZVC;
	SET_FLAGS8(A,t,r);
	A = r;
}

/* $81 CMPA immediate ?**** */
KONAMI_INLINE void cmpa_im( void )
{
	UINT16	  t,r;
	IMMBYTE(t);
	r = A - t;
	CLR_NZVC;
	SET_FLAGS8(A,t,r);
}

/* $82 SBCA immediate ?**** */
KONAMI_INLINE void sbca_im( void )
{
	UINT16	  t,r;
	IMMBYTE(t);
	r = A - t - (CC & CC_C);
	CLR_NZVC;
	SET_FLAGS8(A,t,r);
	A = r;
}

/* $83 SUBD (CMPD CMPU) immediate -**** */
KONAMI_INLINE void subd_im( void )
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
KONAMI_INLINE void cmpd_im( void )
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
KONAMI_INLINE void cmpu_im( void )
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
KONAMI_INLINE void anda_im( void )
{
	UINT8 t;
	IMMBYTE(t);
	A &= t;
	CLR_NZV;
	SET_NZ8(A);
}

/* $85 BITA immediate -**0- */
KONAMI_INLINE void bita_im( void )
{
	UINT8 t,r;
	IMMBYTE(t);
	r = A & t;
	CLR_NZV;
	SET_NZ8(r);
}

/* $86 LDA immediate -**0- */
KONAMI_INLINE void lda_im( void )
{
	IMMBYTE(A);
	CLR_NZV;
	SET_NZ8(A);
}

/* is this a legal instruction? */
/* $87 STA immediate -**0- */
KONAMI_INLINE void sta_im( void )
{
	CLR_NZV;
	SET_NZ8(A);
	IMM8;
	WM(EAD,A);
}

/* $88 EORA immediate -**0- */
KONAMI_INLINE void eora_im( void )
{
	UINT8 t;
	IMMBYTE(t);
	A ^= t;
	CLR_NZV;
	SET_NZ8(A);
}

/* $89 ADCA immediate ***** */
KONAMI_INLINE void adca_im( void )
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
KONAMI_INLINE void ora_im( void )
{
	UINT8 t;
	IMMBYTE(t);
	A |= t;
	CLR_NZV;
	SET_NZ8(A);
}

/* $8B ADDA immediate ***** */
KONAMI_INLINE void adda_im( void )
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
KONAMI_INLINE void cmpx_im( void )
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
KONAMI_INLINE void cmpy_im( void )
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
KONAMI_INLINE void cmps_im( void )
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
KONAMI_INLINE void bsr( void )
{
	UINT8 t;
	IMMBYTE(t);
	PUSHWORD(pPC);
	PC += SIGNED(t);
	change_pc(PCD);
}

/* $8E LDX (LDY) immediate -**0- */
KONAMI_INLINE void ldx_im( void )
{
	IMMWORD(pX);
	CLR_NZV;
	SET_NZ16(X);
}

/* $108E LDY immediate -**0- */
KONAMI_INLINE void ldy_im( void )
{
	IMMWORD(pY);
	CLR_NZV;
	SET_NZ16(Y);
}

/* is this a legal instruction? */
/* $8F STX (STY) immediate -**0- */
KONAMI_INLINE void stx_im( void )
{
	CLR_NZV;
	SET_NZ16(X);
	IMM16;
	WM16(EAD,&pX);
}

/* is this a legal instruction? */
/* $108F STY immediate -**0- */
KONAMI_INLINE void sty_im( void )
{
	CLR_NZV;
	SET_NZ16(Y);
	IMM16;
	WM16(EAD,&pY);
}

/* $90 SUBA direct ?**** */
KONAMI_INLINE void suba_di( void )
{
	UINT16	  t,r;
	DIRBYTE(t);
	r = A - t;
	CLR_NZVC;
	SET_FLAGS8(A,t,r);
	A = r;
}

/* $91 CMPA direct ?**** */
KONAMI_INLINE void cmpa_di( void )
{
	UINT16	  t,r;
	DIRBYTE(t);
	r = A - t;
	CLR_NZVC;
	SET_FLAGS8(A,t,r);
}

/* $92 SBCA direct ?**** */
KONAMI_INLINE void sbca_di( void )
{
	UINT16	  t,r;
	DIRBYTE(t);
	r = A - t - (CC & CC_C);
	CLR_NZVC;
	SET_FLAGS8(A,t,r);
	A = r;
}

/* $93 SUBD (CMPD CMPU) direct -**** */
KONAMI_INLINE void subd_di( void )
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
KONAMI_INLINE void cmpd_di( void )
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
KONAMI_INLINE void cmpu_di( void )
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
KONAMI_INLINE void anda_di( void )
{
	UINT8 t;
	DIRBYTE(t);
	A &= t;
	CLR_NZV;
	SET_NZ8(A);
}

/* $95 BITA direct -**0- */
KONAMI_INLINE void bita_di( void )
{
	UINT8 t,r;
	DIRBYTE(t);
	r = A & t;
	CLR_NZV;
	SET_NZ8(r);
}

/* $96 LDA direct -**0- */
KONAMI_INLINE void lda_di( void )
{
	DIRBYTE(A);
	CLR_NZV;
	SET_NZ8(A);
}

/* $97 STA direct -**0- */
KONAMI_INLINE void sta_di( void )
{
	CLR_NZV;
	SET_NZ8(A);
	DIRECT;
	WM(EAD,A);
}

/* $98 EORA direct -**0- */
KONAMI_INLINE void eora_di( void )
{
	UINT8 t;
	DIRBYTE(t);
	A ^= t;
	CLR_NZV;
	SET_NZ8(A);
}

/* $99 ADCA direct ***** */
KONAMI_INLINE void adca_di( void )
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
KONAMI_INLINE void ora_di( void )
{
	UINT8 t;
	DIRBYTE(t);
	A |= t;
	CLR_NZV;
	SET_NZ8(A);
}

/* $9B ADDA direct ***** */
KONAMI_INLINE void adda_di( void )
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
KONAMI_INLINE void cmpx_di( void )
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
KONAMI_INLINE void cmpy_di( void )
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
KONAMI_INLINE void cmps_di( void )
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
KONAMI_INLINE void jsr_di( void )
{
	DIRECT;
	PUSHWORD(pPC);
	PCD=EAD;
	change_pc(PCD);
}

/* $9E LDX (LDY) direct -**0- */
KONAMI_INLINE void ldx_di( void )
{
	DIRWORD(pX);
	CLR_NZV;
	SET_NZ16(X);
}

/* $109E LDY direct -**0- */
KONAMI_INLINE void ldy_di( void )
{
	DIRWORD(pY);
	CLR_NZV;
	SET_NZ16(Y);
}

/* $9F STX (STY) direct -**0- */
KONAMI_INLINE void stx_di( void )
{
	CLR_NZV;
	SET_NZ16(X);
	DIRECT;
	WM16(EAD,&pX);
}

/* $109F STY direct -**0- */
KONAMI_INLINE void sty_di( void )
{
	CLR_NZV;
	SET_NZ16(Y);
	DIRECT;
	WM16(EAD,&pY);
}

/* $a0 SUBA indexed ?**** */
KONAMI_INLINE void suba_ix( void )
{
	UINT16 t,r;
	t = RM(EAD);
	r = A - t;
	CLR_NZVC;
	SET_FLAGS8(A,t,r);
	A = r;
}

/* $a1 CMPA indexed ?**** */
KONAMI_INLINE void cmpa_ix( void )
{
	UINT16 t,r;
	t = RM(EAD);
	r = A - t;
	CLR_NZVC;
	SET_FLAGS8(A,t,r);
}

/* $a2 SBCA indexed ?**** */
KONAMI_INLINE void sbca_ix( void )
{
	UINT16	  t,r;
	t = RM(EAD);
	r = A - t - (CC & CC_C);
	CLR_NZVC;
	SET_FLAGS8(A,t,r);
	A = r;
}

/* $a3 SUBD (CMPD CMPU) indexed -**** */
KONAMI_INLINE void subd_ix( void )
{
	UINT32 r,d;
	PAIR b;
	b.d=RM16(EAD);
	d = D;
	r = d - b.d;
	CLR_NZVC;
	SET_FLAGS16(d,b.d,r);
	D = r;
}

/* $10a3 CMPD indexed -**** */
KONAMI_INLINE void cmpd_ix( void )
{
	UINT32 r,d;
	PAIR b;
	b.d=RM16(EAD);
	d = D;
	r = d - b.d;
	CLR_NZVC;
	SET_FLAGS16(d,b.d,r);
}

/* $11a3 CMPU indexed -**** */
KONAMI_INLINE void cmpu_ix( void )
{
	UINT32 r;
	PAIR b;
	b.d=RM16(EAD);
	r = U - b.d;
	CLR_NZVC;
	SET_FLAGS16(U,b.d,r);
}

/* $a4 ANDA indexed -**0- */
KONAMI_INLINE void anda_ix( void )
{
	A &= RM(EAD);
	CLR_NZV;
	SET_NZ8(A);
}

/* $a5 BITA indexed -**0- */
KONAMI_INLINE void bita_ix( void )
{
	UINT8 r;
	r = A & RM(EAD);
	CLR_NZV;
	SET_NZ8(r);
}

/* $a6 LDA indexed -**0- */
KONAMI_INLINE void lda_ix( void )
{
	A = RM(EAD);
	CLR_NZV;
	SET_NZ8(A);
}

/* $a7 STA indexed -**0- */
KONAMI_INLINE void sta_ix( void )
{
	CLR_NZV;
	SET_NZ8(A);
	WM(EAD,A);
}

/* $a8 EORA indexed -**0- */
KONAMI_INLINE void eora_ix( void )
{
	A ^= RM(EAD);
	CLR_NZV;
	SET_NZ8(A);
}

/* $a9 ADCA indexed ***** */
KONAMI_INLINE void adca_ix( void )
{
	UINT16 t,r;
	t = RM(EAD);
	r = A + t + (CC & CC_C);
	CLR_HNZVC;
	SET_FLAGS8(A,t,r);
	SET_H(A,t,r);
	A = r;
}

/* $aA ORA indexed -**0- */
KONAMI_INLINE void ora_ix( void )
{
	A |= RM(EAD);
	CLR_NZV;
	SET_NZ8(A);
}

/* $aB ADDA indexed ***** */
KONAMI_INLINE void adda_ix( void )
{
	UINT16 t,r;
	t = RM(EAD);
	r = A + t;
	CLR_HNZVC;
	SET_FLAGS8(A,t,r);
	SET_H(A,t,r);
	A = r;
}

/* $aC CMPX (CMPY CMPS) indexed -**** */
KONAMI_INLINE void cmpx_ix( void )
{
	UINT32 r,d;
	PAIR b;
	b.d=RM16(EAD);
	d = X;
	r = d - b.d;
	CLR_NZVC;
	SET_FLAGS16(d,b.d,r);
}

/* $10aC CMPY indexed -**** */
KONAMI_INLINE void cmpy_ix( void )
{
	UINT32 r,d;
	PAIR b;
	b.d=RM16(EAD);
	d = Y;
	r = d - b.d;
	CLR_NZVC;
	SET_FLAGS16(d,b.d,r);
}

/* $11aC CMPS indexed -**** */
KONAMI_INLINE void cmps_ix( void )
{
	UINT32 r,d;
	PAIR b;
	b.d=RM16(EAD);
	d = S;
	r = d - b.d;
	CLR_NZVC;
	SET_FLAGS16(d,b.d,r);
}

/* $aD JSR indexed ----- */
KONAMI_INLINE void jsr_ix( void )
{
	PUSHWORD(pPC);
	PCD=EAD;
	change_pc(PCD);
}

/* $aE LDX (LDY) indexed -**0- */
KONAMI_INLINE void ldx_ix( void )
{
	X=RM16(EAD);
	CLR_NZV;
	SET_NZ16(X);
}

/* $10aE LDY indexed -**0- */
KONAMI_INLINE void ldy_ix( void )
{
	Y=RM16(EAD);
	CLR_NZV;
	SET_NZ16(Y);
}

/* $aF STX (STY) indexed -**0- */
KONAMI_INLINE void stx_ix( void )
{
	CLR_NZV;
	SET_NZ16(X);
	WM16(EAD,&pX);
}

/* $10aF STY indexed -**0- */
KONAMI_INLINE void sty_ix( void )
{
	CLR_NZV;
	SET_NZ16(Y);
	WM16(EAD,&pY);
}

/* $b0 SUBA extended ?**** */
KONAMI_INLINE void suba_ex( void )
{
	UINT16	  t,r;
	EXTBYTE(t);
	r = A - t;
	CLR_NZVC;
	SET_FLAGS8(A,t,r);
	A = r;
}

/* $b1 CMPA extended ?**** */
KONAMI_INLINE void cmpa_ex( void )
{
	UINT16	  t,r;
	EXTBYTE(t);
	r = A - t;
	CLR_NZVC;
	SET_FLAGS8(A,t,r);
}

/* $b2 SBCA extended ?**** */
KONAMI_INLINE void sbca_ex( void )
{
	UINT16	  t,r;
	EXTBYTE(t);
	r = A - t - (CC & CC_C);
	CLR_NZVC;
	SET_FLAGS8(A,t,r);
	A = r;
}

/* $b3 SUBD (CMPD CMPU) extended -**** */
KONAMI_INLINE void subd_ex( void )
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
KONAMI_INLINE void cmpd_ex( void )
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
KONAMI_INLINE void cmpu_ex( void )
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
KONAMI_INLINE void anda_ex( void )
{
	UINT8 t;
	EXTBYTE(t);
	A &= t;
	CLR_NZV;
	SET_NZ8(A);
}

/* $b5 BITA extended -**0- */
KONAMI_INLINE void bita_ex( void )
{
	UINT8 t,r;
	EXTBYTE(t);
	r = A & t;
	CLR_NZV; SET_NZ8(r);
}

/* $b6 LDA extended -**0- */
KONAMI_INLINE void lda_ex( void )
{
	EXTBYTE(A);
	CLR_NZV;
	SET_NZ8(A);
}

/* $b7 STA extended -**0- */
KONAMI_INLINE void sta_ex( void )
{
	CLR_NZV;
	SET_NZ8(A);
	EXTENDED;
	WM(EAD,A);
}

/* $b8 EORA extended -**0- */
KONAMI_INLINE void eora_ex( void )
{
	UINT8 t;
	EXTBYTE(t);
	A ^= t;
	CLR_NZV;
	SET_NZ8(A);
}

/* $b9 ADCA extended ***** */
KONAMI_INLINE void adca_ex( void )
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
KONAMI_INLINE void ora_ex( void )
{
	UINT8 t;
	EXTBYTE(t);
	A |= t;
	CLR_NZV;
	SET_NZ8(A);
}

/* $bB ADDA extended ***** */
KONAMI_INLINE void adda_ex( void )
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
KONAMI_INLINE void cmpx_ex( void )
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
KONAMI_INLINE void cmpy_ex( void )
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
KONAMI_INLINE void cmps_ex( void )
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
KONAMI_INLINE void jsr_ex( void )
{
	EXTENDED;
	PUSHWORD(pPC);
	PCD=EAD;
	change_pc(PCD);
}

/* $bE LDX (LDY) extended -**0- */
KONAMI_INLINE void ldx_ex( void )
{
	EXTWORD(pX);
	CLR_NZV;
	SET_NZ16(X);
}

/* $10bE LDY extended -**0- */
KONAMI_INLINE void ldy_ex( void )
{
	EXTWORD(pY);
	CLR_NZV;
	SET_NZ16(Y);
}

/* $bF STX (STY) extended -**0- */
KONAMI_INLINE void stx_ex( void )
{
	CLR_NZV;
	SET_NZ16(X);
	EXTENDED;
	WM16(EAD,&pX);
}

/* $10bF STY extended -**0- */
KONAMI_INLINE void sty_ex( void )
{
	CLR_NZV;
	SET_NZ16(Y);
	EXTENDED;
	WM16(EAD,&pY);
}

/* $c0 SUBB immediate ?**** */
KONAMI_INLINE void subb_im( void )
{
	UINT16	  t,r;
	IMMBYTE(t);
	r = B - t;
	CLR_NZVC;
	SET_FLAGS8(B,t,r);
	B = r;
}

/* $c1 CMPB immediate ?**** */
KONAMI_INLINE void cmpb_im( void )
{
	UINT16	  t,r;
	IMMBYTE(t);
	r = B - t;
	CLR_NZVC; SET_FLAGS8(B,t,r);
}

/* $c2 SBCB immediate ?**** */
KONAMI_INLINE void sbcb_im( void )
{
	UINT16	  t,r;
	IMMBYTE(t);
	r = B - t - (CC & CC_C);
	CLR_NZVC;
	SET_FLAGS8(B,t,r);
	B = r;
}

/* $c3 ADDD immediate -**** */
KONAMI_INLINE void addd_im( void )
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
KONAMI_INLINE void andb_im( void )
{
	UINT8 t;
	IMMBYTE(t);
	B &= t;
	CLR_NZV;
	SET_NZ8(B);
}

/* $c5 BITB immediate -**0- */
KONAMI_INLINE void bitb_im( void )
{
	UINT8 t,r;
	IMMBYTE(t);
	r = B & t;
	CLR_NZV;
	SET_NZ8(r);
}

/* $c6 LDB immediate -**0- */
KONAMI_INLINE void ldb_im( void )
{
	IMMBYTE(B);
	CLR_NZV;
	SET_NZ8(B);
}

/* is this a legal instruction? */
/* $c7 STB immediate -**0- */
KONAMI_INLINE void stb_im( void )
{
	CLR_NZV;
	SET_NZ8(B);
	IMM8;
	WM(EAD,B);
}

/* $c8 EORB immediate -**0- */
KONAMI_INLINE void eorb_im( void )
{
	UINT8 t;
	IMMBYTE(t);
	B ^= t;
	CLR_NZV;
	SET_NZ8(B);
}

/* $c9 ADCB immediate ***** */
KONAMI_INLINE void adcb_im( void )
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
KONAMI_INLINE void orb_im( void )
{
	UINT8 t;
	IMMBYTE(t);
	B |= t;
	CLR_NZV;
	SET_NZ8(B);
}

/* $cB ADDB immediate ***** */
KONAMI_INLINE void addb_im( void )
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
KONAMI_INLINE void ldd_im( void )
{
	IMMWORD(pD);
	CLR_NZV;
	SET_NZ16(D);
}

/* is this a legal instruction? */
/* $cD STD immediate -**0- */
KONAMI_INLINE void std_im( void )
{
	CLR_NZV;
	SET_NZ16(D);
    IMM16;
	WM16(EAD,&pD);
}

/* $cE LDU (LDS) immediate -**0- */
KONAMI_INLINE void ldu_im( void )
{
	IMMWORD(pU);
	CLR_NZV;
	SET_NZ16(U);
}

/* $10cE LDS immediate -**0- */
KONAMI_INLINE void lds_im( void )
{
	IMMWORD(pS);
	CLR_NZV;
	SET_NZ16(S);
	konami.int_state |= KONAMI_LDS;
}

/* is this a legal instruction? */
/* $cF STU (STS) immediate -**0- */
KONAMI_INLINE void stu_im( void )
{
	CLR_NZV;
	SET_NZ16(U);
    IMM16;
	WM16(EAD,&pU);
}

/* is this a legal instruction? */
/* $10cF STS immediate -**0- */
KONAMI_INLINE void sts_im( void )
{
	CLR_NZV;
	SET_NZ16(S);
    IMM16;
	WM16(EAD,&pS);
}

/* $d0 SUBB direct ?**** */
KONAMI_INLINE void subb_di( void )
{
	UINT16	  t,r;
	DIRBYTE(t);
	r = B - t;
	CLR_NZVC;
	SET_FLAGS8(B,t,r);
	B = r;
}

/* $d1 CMPB direct ?**** */
KONAMI_INLINE void cmpb_di( void )
{
	UINT16	  t,r;
	DIRBYTE(t);
	r = B - t;
	CLR_NZVC;
	SET_FLAGS8(B,t,r);
}

/* $d2 SBCB direct ?**** */
KONAMI_INLINE void sbcb_di( void )
{
	UINT16	  t,r;
	DIRBYTE(t);
	r = B - t - (CC & CC_C);
	CLR_NZVC;
	SET_FLAGS8(B,t,r);
	B = r;
}

/* $d3 ADDD direct -**** */
KONAMI_INLINE void addd_di( void )
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
KONAMI_INLINE void andb_di( void )
{
	UINT8 t;
	DIRBYTE(t);
	B &= t;
	CLR_NZV;
	SET_NZ8(B);
}

/* $d5 BITB direct -**0- */
KONAMI_INLINE void bitb_di( void )
{
	UINT8 t,r;
	DIRBYTE(t);
	r = B & t;
	CLR_NZV;
	SET_NZ8(r);
}

/* $d6 LDB direct -**0- */
KONAMI_INLINE void ldb_di( void )
{
	DIRBYTE(B);
	CLR_NZV;
	SET_NZ8(B);
}

/* $d7 STB direct -**0- */
KONAMI_INLINE void stb_di( void )
{
	CLR_NZV;
	SET_NZ8(B);
	DIRECT;
	WM(EAD,B);
}

/* $d8 EORB direct -**0- */
KONAMI_INLINE void eorb_di( void )
{
	UINT8 t;
	DIRBYTE(t);
	B ^= t;
	CLR_NZV;
	SET_NZ8(B);
}

/* $d9 ADCB direct ***** */
KONAMI_INLINE void adcb_di( void )
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
KONAMI_INLINE void orb_di( void )
{
	UINT8 t;
	DIRBYTE(t);
	B |= t;
	CLR_NZV;
	SET_NZ8(B);
}

/* $dB ADDB direct ***** */
KONAMI_INLINE void addb_di( void )
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
KONAMI_INLINE void ldd_di( void )
{
	DIRWORD(pD);
	CLR_NZV;
	SET_NZ16(D);
}

/* $dD STD direct -**0- */
KONAMI_INLINE void std_di( void )
{
	CLR_NZV;
	SET_NZ16(D);
    DIRECT;
	WM16(EAD,&pD);
}

/* $dE LDU (LDS) direct -**0- */
KONAMI_INLINE void ldu_di( void )
{
	DIRWORD(pU);
	CLR_NZV;
	SET_NZ16(U);
}

/* $10dE LDS direct -**0- */
KONAMI_INLINE void lds_di( void )
{
	DIRWORD(pS);
	CLR_NZV;
	SET_NZ16(S);
	konami.int_state |= KONAMI_LDS;
}

/* $dF STU (STS) direct -**0- */
KONAMI_INLINE void stu_di( void )
{
	CLR_NZV;
	SET_NZ16(U);
	DIRECT;
	WM16(EAD,&pU);
}

/* $10dF STS direct -**0- */
KONAMI_INLINE void sts_di( void )
{
	CLR_NZV;
	SET_NZ16(S);
	DIRECT;
	WM16(EAD,&pS);
}

/* $e0 SUBB indexed ?**** */
KONAMI_INLINE void subb_ix( void )
{
	UINT16	  t,r;
	t = RM(EAD);
	r = B - t;
	CLR_NZVC;
	SET_FLAGS8(B,t,r);
	B = r;
}

/* $e1 CMPB indexed ?**** */
KONAMI_INLINE void cmpb_ix( void )
{
	UINT16	  t,r;
	t = RM(EAD);
	r = B - t;
	CLR_NZVC;
	SET_FLAGS8(B,t,r);
}

/* $e2 SBCB indexed ?**** */
KONAMI_INLINE void sbcb_ix( void )
{
	UINT16	  t,r;
	t = RM(EAD);
	r = B - t - (CC & CC_C);
	CLR_NZVC;
	SET_FLAGS8(B,t,r);
	B = r;
}

/* $e3 ADDD indexed -**** */
KONAMI_INLINE void addd_ix( void )
{
	UINT32 r,d;
	PAIR b;
	b.d=RM16(EAD);
	d = D;
	r = d + b.d;
	CLR_NZVC;
	SET_FLAGS16(d,b.d,r);
	D = r;
}

/* $e4 ANDB indexed -**0- */
KONAMI_INLINE void andb_ix( void )
{
	B &= RM(EAD);
	CLR_NZV;
	SET_NZ8(B);
}

/* $e5 BITB indexed -**0- */
KONAMI_INLINE void bitb_ix( void )
{
	UINT8 r;
	r = B & RM(EAD);
	CLR_NZV;
	SET_NZ8(r);
}

/* $e6 LDB indexed -**0- */
KONAMI_INLINE void ldb_ix( void )
{
	B = RM(EAD);
	CLR_NZV;
	SET_NZ8(B);
}

/* $e7 STB indexed -**0- */
KONAMI_INLINE void stb_ix( void )
{
	CLR_NZV;
	SET_NZ8(B);
	WM(EAD,B);
}

/* $e8 EORB indexed -**0- */
KONAMI_INLINE void eorb_ix( void )
{
	B ^= RM(EAD);
	CLR_NZV;
	SET_NZ8(B);
}

/* $e9 ADCB indexed ***** */
KONAMI_INLINE void adcb_ix( void )
{
	UINT16 t,r;
	t = RM(EAD);
	r = B + t + (CC & CC_C);
	CLR_HNZVC;
	SET_FLAGS8(B,t,r);
	SET_H(B,t,r);
	B = r;
}

/* $eA ORB indexed -**0- */
KONAMI_INLINE void orb_ix( void )
{
	B |= RM(EAD);
	CLR_NZV;
	SET_NZ8(B);
}

/* $eb ADDB indexed ***** */
KONAMI_INLINE void addb_ix( void )
{
	UINT16 t,r;
	t = RM(EAD);
	r = B + t;
	CLR_HNZVC;
	SET_FLAGS8(B,t,r);
	SET_H(B,t,r);
	B = r;
}

/* $ec LDD indexed -**0- */
KONAMI_INLINE void ldd_ix( void )
{
	D=RM16(EAD);
	CLR_NZV; SET_NZ16(D);
}

/* $eD STD indexed -**0- */
KONAMI_INLINE void std_ix( void )
{
	CLR_NZV;
	SET_NZ16(D);
	WM16(EAD,&pD);
}

/* $eE LDU (LDS) indexed -**0- */
KONAMI_INLINE void ldu_ix( void )
{
	U=RM16(EAD);
	CLR_NZV;
	SET_NZ16(U);
}

/* $10eE LDS indexed -**0- */
KONAMI_INLINE void lds_ix( void )
{
	S=RM16(EAD);
	CLR_NZV;
	SET_NZ16(S);
	konami.int_state |= KONAMI_LDS;
}

/* $eF STU (STS) indexed -**0- */
KONAMI_INLINE void stu_ix( void )
{
	CLR_NZV;
	SET_NZ16(U);
	WM16(EAD,&pU);
}

/* $10eF STS indexed -**0- */
KONAMI_INLINE void sts_ix( void )
{
	CLR_NZV;
	SET_NZ16(S);
	WM16(EAD,&pS);
}

/* $f0 SUBB extended ?**** */
KONAMI_INLINE void subb_ex( void )
{
	UINT16	  t,r;
	EXTBYTE(t);
	r = B - t;
	CLR_NZVC;
	SET_FLAGS8(B,t,r);
	B = r;
}

/* $f1 CMPB extended ?**** */
KONAMI_INLINE void cmpb_ex( void )
{
	UINT16	  t,r;
	EXTBYTE(t);
	r = B - t;
	CLR_NZVC;
	SET_FLAGS8(B,t,r);
}

/* $f2 SBCB extended ?**** */
KONAMI_INLINE void sbcb_ex( void )
{
	UINT16	  t,r;
	EXTBYTE(t);
	r = B - t - (CC & CC_C);
	CLR_NZVC;
	SET_FLAGS8(B,t,r);
	B = r;
}

/* $f3 ADDD extended -**** */
KONAMI_INLINE void addd_ex( void )
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
KONAMI_INLINE void andb_ex( void )
{
	UINT8 t;
	EXTBYTE(t);
	B &= t;
	CLR_NZV;
	SET_NZ8(B);
}

/* $f5 BITB extended -**0- */
KONAMI_INLINE void bitb_ex( void )
{
	UINT8 t,r;
	EXTBYTE(t);
	r = B & t;
	CLR_NZV;
	SET_NZ8(r);
}

/* $f6 LDB extended -**0- */
KONAMI_INLINE void ldb_ex( void )
{
	EXTBYTE(B);
	CLR_NZV;
	SET_NZ8(B);
}

/* $f7 STB extended -**0- */
KONAMI_INLINE void stb_ex( void )
{
	CLR_NZV;
	SET_NZ8(B);
	EXTENDED;
	WM(EAD,B);
}

/* $f8 EORB extended -**0- */
KONAMI_INLINE void eorb_ex( void )
{
	UINT8 t;
	EXTBYTE(t);
	B ^= t;
	CLR_NZV;
	SET_NZ8(B);
}

/* $f9 ADCB extended ***** */
KONAMI_INLINE void adcb_ex( void )
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
KONAMI_INLINE void orb_ex( void )
{
	UINT8 t;
	EXTBYTE(t);
	B |= t;
	CLR_NZV;
	SET_NZ8(B);
}

/* $fB ADDB extended ***** */
KONAMI_INLINE void addb_ex( void )
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
KONAMI_INLINE void ldd_ex( void )
{
	EXTWORD(pD);
	CLR_NZV;
	SET_NZ16(D);
}

/* $fD STD extended -**0- */
KONAMI_INLINE void std_ex( void )
{
	CLR_NZV;
	SET_NZ16(D);
    EXTENDED;
	WM16(EAD,&pD);
}

/* $fE LDU (LDS) extended -**0- */
KONAMI_INLINE void ldu_ex( void )
{
	EXTWORD(pU);
	CLR_NZV;
	SET_NZ16(U);
}

/* $10fE LDS extended -**0- */
KONAMI_INLINE void lds_ex( void )
{
	EXTWORD(pS);
	CLR_NZV;
	SET_NZ16(S);
	konami.int_state |= KONAMI_LDS;
}

/* $fF STU (STS) extended -**0- */
KONAMI_INLINE void stu_ex( void )
{
	CLR_NZV;
	SET_NZ16(U);
	EXTENDED;
	WM16(EAD,&pU);
}

/* $10fF STS extended -**0- */
KONAMI_INLINE void sts_ex( void )
{
	CLR_NZV;
	SET_NZ16(S);
	EXTENDED;
	WM16(EAD,&pS);
}

KONAMI_INLINE void setline_im( void )
{
	UINT8 t;
	IMMBYTE(t);

	if ( konami.setlines_callback )
		(*konami.setlines_callback)( t );
}

KONAMI_INLINE void setline_ix( void )
{
	UINT8 t;
	t = RM(EA);

	if ( konami.setlines_callback )
		(*konami.setlines_callback)( t );
}

KONAMI_INLINE void setline_di( void )
{
	UINT8 t;
	DIRBYTE(t);

	if ( konami.setlines_callback )
		(*konami.setlines_callback)( t );
}

KONAMI_INLINE void setline_ex( void )
{
	UINT8 t;
	EXTBYTE(t);

	if ( konami.setlines_callback )
		(*konami.setlines_callback)( t );
}

KONAMI_INLINE void bmove( void )
{
	UINT8	t;

	while( U != 0 ) {
		t = RM(Y);
		WM(X,t);
		Y++;
		X++;
		U--;
		konami_ICount -= 2;
	}
}

KONAMI_INLINE void move( void )
{
	UINT8	t;

	t = RM(Y);
	WM(X,t);
	Y++;
	X++;
	U--;
}

/* CLRD inherent -0100 */
KONAMI_INLINE void clrd( void )
{
	D = 0;
	CLR_NZVC; SEZ;
}

/* CLRW indexed -0100 */
KONAMI_INLINE void clrw_ix( void )
{
	PAIR t;
	t.d = 0;
	WM16(EAD,&t);
	CLR_NZVC; SEZ;
}

/* CLRW direct -0100 */
KONAMI_INLINE void clrw_di( void )
{
	PAIR t;
	t.d = 0;
	DIRECT;
	WM16(EAD,&t);
	CLR_NZVC;
	SEZ;
}

/* CLRW extended -0100 */
KONAMI_INLINE void clrw_ex( void )
{
	PAIR t;
	t.d = 0;
	EXTENDED;
	WM16(EAD,&t);
	CLR_NZVC; SEZ;
}

/* LSRD immediate -0*-* */
KONAMI_INLINE void lsrd( void )
{
	UINT8 t;

	IMMBYTE( t );

	while ( t-- ) {
		CLR_NZC;
		CC |= (D & CC_C);
		D >>= 1;
		SET_Z16(D);
	}
}

/* RORD immediate -**-* */
KONAMI_INLINE void rord( void )
{
	UINT16 r;
	UINT8  t;

	IMMBYTE(t);

	while ( t-- ) {
		r = (CC & CC_C) << 15;
		CLR_NZC;
		CC |= (D & CC_C);
		r |= D >> 1;
		SET_NZ16(r);
		D = r;
	}
}

/* ASRD immediate ?**-* */
KONAMI_INLINE void asrd( void )
{
	UINT8 t;

	IMMBYTE(t);

	while ( t-- ) {
		CLR_NZC;
		CC |= (D & CC_C);
		D = (D & 0x8000) | (D >> 1);
		SET_NZ16(D);
	}
}

/* ASLD immediate ?**** */
KONAMI_INLINE void asld( void )
{
	UINT32	r;
	UINT8	t;

	IMMBYTE( t );

	while ( t-- ) {
		r = D << 1;
		CLR_NZVC;
		SET_FLAGS16(D,D,r);
		D = r;
	}
}

/* ROLD immediate -**-* */
KONAMI_INLINE void rold( void )
{
	UINT16 r;
	UINT8  t;

	IMMBYTE(t);

	while ( t-- ) {
		CLR_NZC;
		if ( D & 0x8000 ) SEC;
		r = CC & CC_C;
		r |= D << 1;
		SET_NZ16(r);
		D = r;
	}
}

/* DECB,JNZ relative ----- */
KONAMI_INLINE void decbjnz( void )
{
	--B;
	CLR_NZV;
	SET_FLAGS8D(B);
	BRANCH( !(CC&CC_Z) );
}

/* DECX,JNZ relative ----- */
KONAMI_INLINE void decxjnz( void )
{
	--X;
	CLR_NZV;
	SET_NZ16(X);	/* should affect V as well? */
	BRANCH( !(CC&CC_Z) );
}

KONAMI_INLINE void bset( void )
{
	UINT8	t;

	while( U != 0 ) {
		t = A;
		WM(XD,t);
		X++;
		U--;
		konami_ICount -= 2;
	}
}

KONAMI_INLINE void bset2( void )
{
	while( U != 0 ) {
		WM16(XD,&pD);
		X += 2;
		U--;
		konami_ICount -= 3;
	}
}

/* LMUL inherent --*-@ */
KONAMI_INLINE void lmul( void )
{
	UINT32 t;
	t = X * Y;
	X = (t >> 16);
	Y = (t & 0xffff);
	CLR_ZC; SET_Z(t); if( t & 0x8000 ) SEC;
}

/* DIVX inherent --*-@ */
KONAMI_INLINE void divx( void )
{
	UINT16 t;
	UINT8 r;
	if ( B != 0 )
	{
		t = X / B;
		r = X % B;
	}
	else
	{
		/* ?? */
		t = 0;
		r = 0;
	}
	CLR_ZC; SET_Z16(t); if ( t & 0x80 ) SEC;
	X = t;
	B = r;
}

/* INCD inherent -***- */
KONAMI_INLINE void incd( void )
{
	UINT32 r;
	r = D + 1;
	CLR_NZV;
	SET_FLAGS16(D,D,r);
	D = r;
}

/* INCW direct -***- */
KONAMI_INLINE void incw_di( void )
{
	PAIR t,r;
	DIRWORD(t);
	r = t;
	++r.d;
	CLR_NZV;
	SET_FLAGS16(t.d, t.d, r.d);
	WM16(EAD,&r);
}

/* INCW indexed -***- */
KONAMI_INLINE void incw_ix( void )
{
	PAIR t,r;
	t.d=RM16(EAD);
	r = t;
	++r.d;
	CLR_NZV;
	SET_FLAGS16(t.d, t.d, r.d);
	WM16(EAD,&r);
}

/* INCW extended -***- */
KONAMI_INLINE void incw_ex( void )
{
	PAIR t, r;
	EXTWORD(t);
	r = t;
	++r.d;
	CLR_NZV; SET_FLAGS16(t.d,t.d,r.d);
	WM16(EAD,&r);
}

/* DECD inherent -***- */
KONAMI_INLINE void decd( void )
{
	UINT32 r;
	r = D - 1;
	CLR_NZV;
	SET_FLAGS16(D,D,r);
	D = r;
}

/* DECW direct -***- */
KONAMI_INLINE void decw_di( void )
{
	PAIR t,r;
	DIRWORD(t);
	r = t;
	--r.d;
	CLR_NZV;
	SET_FLAGS16(t.d, t.d, r.d);
	WM16(EAD,&r);
}

/* DECW indexed -***- */
KONAMI_INLINE void decw_ix( void )
{
	PAIR t, r;
	t.d=RM16(EAD);
	r = t;
	--r.d;
	CLR_NZV; SET_FLAGS16(t.d, t.d, r.d);
	WM16(EAD,&r);
}

/* DECW extended -***- */
KONAMI_INLINE void decw_ex( void )
{
	PAIR t, r;
	EXTWORD(t);
	r = t;
	--r.d;
	CLR_NZV; SET_FLAGS16(t.d,t.d,r.d);
	WM16(EAD,&r);
}

/* TSTD inherent -**0- */
KONAMI_INLINE void tstd( void )
{
	CLR_NZV;
	SET_NZ16(D);
}

/* TSTW direct -**0- */
KONAMI_INLINE void tstw_di( void )
{
	PAIR t;
	CLR_NZV;
	DIRWORD(t);
	SET_NZ16(t.d);
}

/* TSTW indexed -**0- */
KONAMI_INLINE void tstw_ix( void )
{
	PAIR t;
	CLR_NZV;
	t.d=RM16(EAD);
	SET_NZ16(t.d);
}

/* TSTW extended -**0- */
KONAMI_INLINE void tstw_ex( void )
{
	PAIR t;
	CLR_NZV;
	EXTWORD(t);
	SET_NZ16(t.d);
}

/* LSRW direct -0*-* */
KONAMI_INLINE void lsrw_di( void )
{
	PAIR t;
	DIRWORD(t);
	CLR_NZC;
	CC |= (t.d & CC_C);
	t.d >>= 1;
	SET_Z16(t.d);
	WM16(EAD,&t);
}

/* LSRW indexed -0*-* */
KONAMI_INLINE void lsrw_ix( void )
{
	PAIR t;
	t.d=RM16(EAD);
	CLR_NZC;
	CC |= (t.d & CC_C);
	t.d >>= 1;
	SET_Z16(t.d);
	WM16(EAD,&t);
}

/* LSRW extended -0*-* */
KONAMI_INLINE void lsrw_ex( void )
{
	PAIR t;
	EXTWORD(t);
	CLR_NZC;
	CC |= (t.d & CC_C);
	t.d >>= 1;
	SET_Z16(t.d);
	WM16(EAD,&t);
}

/* RORW direct -**-* */
KONAMI_INLINE void rorw_di( void )
{
	PAIR t,r;
	DIRWORD(t);
	r.d = (CC & CC_C) << 15;
	CLR_NZC;
	CC |= (t.d & CC_C);
	r.d |= t.d>>1;
	SET_NZ16(r.d);
	WM16(EAD,&r);
}

/* RORW indexed -**-* */
KONAMI_INLINE void rorw_ix( void )
{
	PAIR t,r;
	t.d=RM16(EAD);
	r.d = (CC & CC_C) << 15;
	CLR_NZC;
	CC |= (t.d & CC_C);
	r.d |= t.d>>1;
	SET_NZ16(r.d);
	WM16(EAD,&r);
}

/* RORW extended -**-* */
KONAMI_INLINE void rorw_ex( void )
{
	PAIR t,r;
	EXTWORD(t);
	r.d = (CC & CC_C) << 15;
	CLR_NZC;
	CC |= (t.d & CC_C);
	r.d |= t.d>>1;
	SET_NZ16(r.d);
	WM16(EAD,&r);
}

/* ASRW direct ?**-* */
KONAMI_INLINE void asrw_di( void )
{
	PAIR t;
	DIRWORD(t);
	CLR_NZC;
	CC |= (t.d & CC_C);
	t.d = (t.d & 0x8000) | (t.d >> 1);
	SET_NZ16(t.d);
	WM16(EAD,&t);
}

/* ASRW indexed ?**-* */
KONAMI_INLINE void asrw_ix( void )
{
	PAIR t;
	t.d=RM16(EAD);
	CLR_NZC;
	CC |= (t.d & CC_C);
	t.d = (t.d & 0x8000) | (t.d >> 1);
	SET_NZ16(t.d);
	WM16(EAD,&t);
}

/* ASRW extended ?**-* */
KONAMI_INLINE void asrw_ex( void )
{
	PAIR t;
	EXTWORD(t);
	CLR_NZC;
	CC |= (t.d & CC_C);
	t.d = (t.d & 0x8000) | (t.d >> 1);
	SET_NZ16(t.d);
	WM16(EAD,&t);
}

/* ASLW direct ?**** */
KONAMI_INLINE void aslw_di( void )
{
	PAIR t,r;
	DIRWORD(t);
	r.d = t.d << 1;
	CLR_NZVC;
	SET_FLAGS16(t.d,t.d,r.d);
	WM16(EAD,&r);
}

/* ASLW indexed ?**** */
KONAMI_INLINE void aslw_ix( void )
{
	PAIR t,r;
	t.d=RM16(EAD);
	r.d = t.d << 1;
	CLR_NZVC;
	SET_FLAGS16(t.d,t.d,r.d);
	WM16(EAD,&r);
}

/* ASLW extended ?**** */
KONAMI_INLINE void aslw_ex( void )
{
	PAIR t,r;
	EXTWORD(t);
	r.d = t.d << 1;
	CLR_NZVC;
	SET_FLAGS16(t.d,t.d,r.d);
	WM16(EAD,&r);
}

/* ROLW direct -**** */
KONAMI_INLINE void rolw_di( void )
{
	PAIR t,r;
	DIRWORD(t);
	r.d = (CC & CC_C) | (t.d << 1);
	CLR_NZVC;
	SET_FLAGS16(t.d,t.d,r.d);
	WM16(EAD,&r);
}

/* ROLW indexed -**** */
KONAMI_INLINE void rolw_ix( void )
{
	PAIR t,r;
	t.d=RM16(EAD);
	r.d = (CC & CC_C) | (t.d << 1);
	CLR_NZVC;
	SET_FLAGS16(t.d,t.d,r.d);
	WM16(EAD,&r);
}

/* ROLW extended -**** */
KONAMI_INLINE void rolw_ex( void )
{
	PAIR t,r;
	EXTWORD(t);
	r.d = (CC & CC_C) | (t.d << 1);
	CLR_NZVC;
	SET_FLAGS16(t.d,t.d,r.d);
	WM16(EAD,&r);
}

/* NEGD inherent ?**** */
KONAMI_INLINE void negd( void )
{
	UINT32 r;
	r = -D;
	CLR_NZVC;
	SET_FLAGS16(0,D,r);
	D = r;
}

/* NEGW direct ?**** */
KONAMI_INLINE void negw_di( void )
{
	PAIR r,t;
	DIRWORD(t);
	r.d = -t.d;
	CLR_NZVC;
	SET_FLAGS16(0,t.d,r.d);
	WM16(EAD,&r);
}

/* NEGW indexed ?**** */
KONAMI_INLINE void negw_ix( void )
{
	PAIR r,t;
	t.d=RM16(EAD);
	r.d = -t.d;
	CLR_NZVC;
	SET_FLAGS16(0,t.d,r.d);
	WM16(EAD,&r);
}

/* NEGW extended ?**** */
KONAMI_INLINE void negw_ex( void )
{
	PAIR r,t;
	EXTWORD(t);
	r.d = -t.d;
	CLR_NZVC;
	SET_FLAGS16(0,t.d,r.d);
	WM16(EAD,&r);
}

/* ABSA inherent ?**** */
KONAMI_INLINE void absa( void )
{
	UINT16 r;
	if (A & 0x80)
		r = -A;
	else
		r = A;
	CLR_NZVC;
	SET_FLAGS8(0,A,r);
	A = r;
}

/* ABSB inherent ?**** */
KONAMI_INLINE void absb( void )
{
	UINT16 r;
	if (B & 0x80)
		r = -B;
	else
		r = B;
	CLR_NZVC;
	SET_FLAGS8(0,B,r);
	B = r;
}

/* ABSD inherent ?**** */
KONAMI_INLINE void absd( void )
{
	UINT32 r;
	if (D & 0x8000)
		r = -D;
	else
		r = D;
	CLR_NZVC;
	SET_FLAGS16(0,D,r);
	D = r;
}

/* LSRD direct -0*-* */
KONAMI_INLINE void lsrd_di( void )
{
	UINT8 t;

	DIRBYTE( t );

	while ( t-- ) {
		CLR_NZC;
		CC |= (D & CC_C);
		D >>= 1;
		SET_Z16(D);
	}
}

/* RORD direct -**-* */
KONAMI_INLINE void rord_di( void )
{
	UINT16 r;
	UINT8  t;

	DIRBYTE(t);

	while ( t-- ) {
		r = (CC & CC_C) << 15;
		CLR_NZC;
		CC |= (D & CC_C);
		r |= D >> 1;
		SET_NZ16(r);
		D = r;
	}
}

/* ASRD direct ?**-* */
KONAMI_INLINE void asrd_di( void )
{
	UINT8 t;

	DIRBYTE(t);

	while ( t-- ) {
		CLR_NZC;
		CC |= (D & CC_C);
		D = (D & 0x8000) | (D >> 1);
		SET_NZ16(D);
	}
}

/* ASLD direct ?**** */
KONAMI_INLINE void asld_di( void )
{
	UINT32	r;
	UINT8	t;

	DIRBYTE( t );

	while ( t-- ) {
		r = D << 1;
		CLR_NZVC;
		SET_FLAGS16(D,D,r);
		D = r;
	}
}

/* ROLD direct -**-* */
KONAMI_INLINE void rold_di( void )
{
	UINT16 r;
	UINT8  t;

	DIRBYTE(t);

	while ( t-- ) {
		CLR_NZC;
		if ( D & 0x8000 ) SEC;
		r = CC & CC_C;
		r |= D << 1;
		SET_NZ16(r);
		D = r;
	}
}

/* LSRD indexed -0*-* */
KONAMI_INLINE void lsrd_ix( void )
{
	UINT8 t;

	t=RM(EA);

	while ( t-- ) {
		CLR_NZC;
		CC |= (D & CC_C);
		D >>= 1;
		SET_Z16(D);
	}
}

/* RORD indexed -**-* */
KONAMI_INLINE void rord_ix( void )
{
	UINT16 r;
	UINT8  t;

	t=RM(EA);

	while ( t-- ) {
		r = (CC & CC_C) << 15;
		CLR_NZC;
		CC |= (D & CC_C);
		r |= D >> 1;
		SET_NZ16(r);
		D = r;
	}
}

/* ASRD indexed ?**-* */
KONAMI_INLINE void asrd_ix( void )
{
	UINT8 t;

	t=RM(EA);

	while ( t-- ) {
		CLR_NZC;
		CC |= (D & CC_C);
		D = (D & 0x8000) | (D >> 1);
		SET_NZ16(D);
	}
}

/* ASLD indexed ?**** */
KONAMI_INLINE void asld_ix( void )
{
	UINT32	r;
	UINT8	t;

	t=RM(EA);

	while ( t-- ) {
		r = D << 1;
		CLR_NZVC;
		SET_FLAGS16(D,D,r);
		D = r;
	}
}

/* ROLD indexed -**-* */
KONAMI_INLINE void rold_ix( void )
{
	UINT16 r;
	UINT8  t;

	t=RM(EA);

	while ( t-- ) {
		CLR_NZC;
		if ( D & 0x8000 ) SEC;
		r = CC & CC_C;
		r |= D << 1;
		SET_NZ16(r);
		D = r;
	}
}

/* LSRD extended -0*-* */
KONAMI_INLINE void lsrd_ex( void )
{
	UINT8 t;

	EXTBYTE(t);

	while ( t-- ) {
		CLR_NZC;
		CC |= (D & CC_C);
		D >>= 1;
		SET_Z16(D);
	}
}

/* RORD extended -**-* */
KONAMI_INLINE void rord_ex( void )
{
	UINT16 r;
	UINT8  t;

	EXTBYTE(t);

	while ( t-- ) {
		r = (CC & CC_C) << 15;
		CLR_NZC;
		CC |= (D & CC_C);
		r |= D >> 1;
		SET_NZ16(r);
		D = r;
	}
}

/* ASRD extended ?**-* */
KONAMI_INLINE void asrd_ex( void )
{
	UINT8 t;

	EXTBYTE(t);

	while ( t-- ) {
		CLR_NZC;
		CC |= (D & CC_C);
		D = (D & 0x8000) | (D >> 1);
		SET_NZ16(D);
	}
}

/* ASLD extended ?**** */
KONAMI_INLINE void asld_ex( void )
{
	UINT32	r;
	UINT8	t;

	EXTBYTE(t);

	while ( t-- ) {
		r = D << 1;
		CLR_NZVC;
		SET_FLAGS16(D,D,r);
		D = r;
	}
}

/* ROLD extended -**-* */
KONAMI_INLINE void rold_ex( void )
{
	UINT16 r;
	UINT8  t;

	EXTBYTE(t);

	while ( t-- ) {
		CLR_NZC;
		if ( D & 0x8000 ) SEC;
		r = CC & CC_C;
		r |= D << 1;
		SET_NZ16(r);
		D = r;
	}
}

KONAMI_INLINE void opcode2( void )
{
	UINT8 ireg2 = ROP_ARG(PCD);
	PC++;

	switch ( ireg2 ) {
//  case 0x00: EA=0; break; /* auto increment */
//  case 0x01: EA=0; break; /* double auto increment */
//  case 0x02: EA=0; break; /* auto decrement */
//  case 0x03: EA=0; break; /* double auto decrement */
//  case 0x04: EA=0; break; /* postbyte offs */
//  case 0x05: EA=0; break; /* postword offs */
//  case 0x06: EA=0; break; /* normal */
	case 0x07:
		EAD=0;
		(*konami_extended[konami.ireg])();
        konami_ICount -= 2;
		return;
//  case 0x08: EA=0; break; /* indirect - auto increment */
//  case 0x09: EA=0; break; /* indirect - double auto increment */
//  case 0x0a: EA=0; break; /* indirect - auto decrement */
//  case 0x0b: EA=0; break; /* indirect - double auto decrement */
//  case 0x0c: EA=0; break; /* indirect - postbyte offs */
//  case 0x0d: EA=0; break; /* indirect - postword offs */
//  case 0x0e: EA=0; break; /* indirect - normal */
	case 0x0f:				/* indirect - extended */
		IMMWORD(ea);
		EA=RM16(EAD);
        konami_ICount-=4;
		break;
//  case 0x10: EA=0; break; /* auto increment */
//  case 0x11: EA=0; break; /* double auto increment */
//  case 0x12: EA=0; break; /* auto decrement */
//  case 0x13: EA=0; break; /* double auto decrement */
//  case 0x14: EA=0; break; /* postbyte offs */
//  case 0x15: EA=0; break; /* postword offs */
//  case 0x16: EA=0; break; /* normal */
//  case 0x17: EA=0; break; /* extended */
//  case 0x18: EA=0; break; /* indirect - auto increment */
//  case 0x19: EA=0; break; /* indirect - double auto increment */
//  case 0x1a: EA=0; break; /* indirect - auto decrement */
//  case 0x1b: EA=0; break; /* indirect - double auto decrement */
//  case 0x1c: EA=0; break; /* indirect - postbyte offs */
//  case 0x1d: EA=0; break; /* indirect - postword offs */
//  case 0x1e: EA=0; break; /* indirect - normal */
//  case 0x1f: EA=0; break; /* indirect - extended */

/* base X */
    case 0x20:              /* auto increment */
		EA=X;
		X++;
        konami_ICount-=2;
		break;
	case 0x21:				/* double auto increment */
		EA=X;
		X+=2;
        konami_ICount-=3;
        break;
	case 0x22:				/* auto decrement */
		X--;
		EA=X;
        konami_ICount-=2;
        break;
	case 0x23:				/* double auto decrement */
		X-=2;
		EA=X;
        konami_ICount-=3;
		break;
	case 0x24:				/* postbyte offs */
		IMMBYTE(EA);
		EA=X+SIGNED(EA);
        konami_ICount-=2;
		break;
	case 0x25:				/* postword offs */
		IMMWORD(ea);
		EA+=X;
        konami_ICount-=4;
		break;
	case 0x26:				/* normal */
		EA=X;
		break;
//  case 0x27: EA=0; break; /* extended */
	case 0x28:				/* indirect - auto increment */
		EA=X;
		X++;
		EA=RM16(EAD);
        konami_ICount-=5;
		break;
	case 0x29:				/* indirect - double auto increment */
		EA=X;
		X+=2;
		EA=RM16(EAD);
        konami_ICount-=6;
		break;
	case 0x2a:				/* indirect - auto decrement */
		X--;
		EA=X;
		EA=RM16(EAD);
        konami_ICount-=5;
		break;
	case 0x2b:				/* indirect - double auto decrement */
		X-=2;
		EA=X;
		EA=RM16(EAD);
        konami_ICount-=6;
		break;
	case 0x2c:				/* indirect - postbyte offs */
		IMMBYTE(EA);
		EA=X+SIGNED(EA);
		EA=RM16(EAD);
        konami_ICount-=4;
		break;
	case 0x2d:				/* indirect - postword offs */
		IMMWORD(ea);
		EA+=X;
		EA=RM16(EAD);
        konami_ICount-=7;
		break;
	case 0x2e:				/* indirect - normal */
		EA=X;
		EA=RM16(EAD);
        konami_ICount-=3;
		break;
//  case 0x2f: EA=0; break; /* indirect - extended */

/* base Y */
    case 0x30:              /* auto increment */
		EA=Y;
		Y++;
        konami_ICount-=2;
		break;
	case 0x31:				/* double auto increment */
		EA=Y;
		Y+=2;
        konami_ICount-=3;
		break;
	case 0x32:				/* auto decrement */
		Y--;
		EA=Y;
        konami_ICount-=2;
		break;
	case 0x33:				/* double auto decrement */
		Y-=2;
		EA=Y;
        konami_ICount-=3;
		break;
	case 0x34:				/* postbyte offs */
		IMMBYTE(EA);
		EA=Y+SIGNED(EA);
        konami_ICount-=2;
		break;
	case 0x35:				/* postword offs */
		IMMWORD(ea);
		EA+=Y;
        konami_ICount-=4;
		break;
	case 0x36:				/* normal */
		EA=Y;
		break;
//  case 0x37: EA=0; break; /* extended */
	case 0x38:				/* indirect - auto increment */
		EA=Y;
		Y++;
		EA=RM16(EAD);
        konami_ICount-=5;
		break;
	case 0x39:				/* indirect - double auto increment */
		EA=Y;
		Y+=2;
		EA=RM16(EAD);
        konami_ICount-=6;
		break;
	case 0x3a:				/* indirect - auto decrement */
		Y--;
		EA=Y;
		EA=RM16(EAD);
        konami_ICount-=5;
		break;
	case 0x3b:				/* indirect - double auto decrement */
		Y-=2;
		EA=Y;
		EA=RM16(EAD);
        konami_ICount-=6;
		break;
	case 0x3c:				/* indirect - postbyte offs */
		IMMBYTE(EA);
		EA=Y+SIGNED(EA);
		EA=RM16(EAD);
        konami_ICount-=4;
		break;
	case 0x3d:				/* indirect - postword offs */
		IMMWORD(ea);
		EA+=Y;
		EA=RM16(EAD);
        konami_ICount-=7;
		break;
	case 0x3e:				/* indirect - normal */
		EA=Y;
		EA=RM16(EAD);
        konami_ICount-=3;
		break;
//  case 0x3f: EA=0; break; /* indirect - extended */

//  case 0x40: EA=0; break; /* auto increment */
//  case 0x41: EA=0; break; /* double auto increment */
//  case 0x42: EA=0; break; /* auto decrement */
//  case 0x43: EA=0; break; /* double auto decrement */
//  case 0x44: EA=0; break; /* postbyte offs */
//  case 0x45: EA=0; break; /* postword offs */
//  case 0x46: EA=0; break; /* normal */
//  case 0x47: EA=0; break; /* extended */
//  case 0x48: EA=0; break; /* indirect - auto increment */
//  case 0x49: EA=0; break; /* indirect - double auto increment */
//  case 0x4a: EA=0; break; /* indirect - auto decrement */
//  case 0x4b: EA=0; break; /* indirect - double auto decrement */
//  case 0x4c: EA=0; break; /* indirect - postbyte offs */
//  case 0x4d: EA=0; break; /* indirect - postword offs */
//  case 0x4e: EA=0; break; /* indirect - normal */
//  case 0x4f: EA=0; break; /* indirect - extended */

/* base U */
    case 0x50:              /* auto increment */
		EA=U;
		U++;
        konami_ICount-=2;
		break;
	case 0x51:				/* double auto increment */
		EA=U;
		U+=2;
        konami_ICount-=3;
		break;
	case 0x52:				/* auto decrement */
		U--;
		EA=U;
        konami_ICount-=2;
		break;
	case 0x53:				/* double auto decrement */
		U-=2;
		EA=U;
        konami_ICount-=3;
		break;
	case 0x54:				/* postbyte offs */
		IMMBYTE(EA);
		EA=U+SIGNED(EA);
        konami_ICount-=2;
		break;
	case 0x55:				/* postword offs */
		IMMWORD(ea);
		EA+=U;
        konami_ICount-=4;
		break;
	case 0x56:				/* normal */
		EA=U;
		break;
//  case 0x57: EA=0; break; /* extended */
	case 0x58:				/* indirect - auto increment */
		EA=U;
		U++;
		EA=RM16(EAD);
        konami_ICount-=5;
		break;
	case 0x59:				/* indirect - double auto increment */
		EA=U;
		U+=2;
		EA=RM16(EAD);
        konami_ICount-=6;
		break;
	case 0x5a:				/* indirect - auto decrement */
		U--;
		EA=U;
		EA=RM16(EAD);
        konami_ICount-=5;
		break;
	case 0x5b:				/* indirect - double auto decrement */
		U-=2;
		EA=U;
		EA=RM16(EAD);
        konami_ICount-=6;
		break;
	case 0x5c:				/* indirect - postbyte offs */
		IMMBYTE(EA);
		EA=U+SIGNED(EA);
		EA=RM16(EAD);
        konami_ICount-=4;
		break;
	case 0x5d:				/* indirect - postword offs */
		IMMWORD(ea);
		EA+=U;
		EA=RM16(EAD);
        konami_ICount-=7;
		break;
	case 0x5e:				/* indirect - normal */
		EA=U;
		EA=RM16(EAD);
        konami_ICount-=3;
		break;
//  case 0x5f: EA=0; break; /* indirect - extended */

/* base S */
    case 0x60:              /* auto increment */
		EAD=SD;
		S++;
        konami_ICount-=2;
		break;
	case 0x61:				/* double auto increment */
		EAD=SD;
		S+=2;
        konami_ICount-=3;
		break;
	case 0x62:				/* auto decrement */
		S--;
		EAD=SD;
        konami_ICount-=2;
		break;
	case 0x63:				/* double auto decrement */
		S-=2;
		EAD=SD;
        konami_ICount-=3;
		break;
	case 0x64:				/* postbyte offs */
		IMMBYTE(EA);
		EA=S+SIGNED(EA);
        konami_ICount-=2;
		break;
	case 0x65:				/* postword offs */
		IMMWORD(ea);
		EA+=S;
        konami_ICount-=4;
		break;
	case 0x66:				/* normal */
		EAD=SD;
		break;
//  case 0x67: EA=0; break; /* extended */
	case 0x68:				/* indirect - auto increment */
		EAD=SD;
		S++;
		EA=RM16(EAD);
        konami_ICount-=5;
		break;
	case 0x69:				/* indirect - double auto increment */
		EAD=SD;
		S+=2;
		EA=RM16(EAD);
        konami_ICount-=6;
		break;
	case 0x6a:				/* indirect - auto decrement */
		S--;
		EAD=SD;
		EA=RM16(EAD);
        konami_ICount-=5;
		break;
	case 0x6b:				/* indirect - double auto decrement */
		S-=2;
		EAD=SD;
		EA=RM16(EAD);
        konami_ICount-=6;
		break;
	case 0x6c:				/* indirect - postbyte offs */
		IMMBYTE(EA);
		EA=S+SIGNED(EA);
		EA=RM16(EAD);
        konami_ICount-=4;
		break;
	case 0x6d:				/* indirect - postword offs */
		IMMWORD(ea);
		EA+=S;
		EA=RM16(EAD);
        konami_ICount-=7;
		break;
	case 0x6e:				/* indirect - normal */
		EAD=SD;
		EA=RM16(EAD);
        konami_ICount-=3;
		break;
//  case 0x6f: EA=0; break; /* indirect - extended */

/* base PC */
    case 0x70:              /* auto increment */
		EAD=PCD;
		PC++;
        konami_ICount-=2;
		break;
	case 0x71:				/* double auto increment */
		EAD=PCD;
		PC+=2;
        konami_ICount-=3;
		break;
	case 0x72:				/* auto decrement */
		PC--;
		EAD=PCD;
        konami_ICount-=2;
		break;
	case 0x73:				/* double auto decrement */
		PC-=2;
		EAD=PCD;
        konami_ICount-=3;
		break;
	case 0x74:				/* postbyte offs */
		IMMBYTE(EA);
		EA=PC-1+SIGNED(EA);
        konami_ICount-=2;
		break;
	case 0x75:				/* postword offs */
		IMMWORD(ea);
		EA+=PC-2;
        konami_ICount-=4;
		break;
	case 0x76:				/* normal */
		EAD=PCD;
		break;
//  case 0x77: EA=0; break; /* extended */
	case 0x78:				/* indirect - auto increment */
		EAD=PCD;
		PC++;
		EA=RM16(EAD);
        konami_ICount-=5;
		break;
	case 0x79:				/* indirect - double auto increment */
		EAD=PCD;
		PC+=2;
		EA=RM16(EAD);
        konami_ICount-=6;
		break;
	case 0x7a:				/* indirect - auto decrement */
		PC--;
		EAD=PCD;
		EA=RM16(EAD);
        konami_ICount-=5;
		break;
	case 0x7b:				/* indirect - double auto decrement */
		PC-=2;
		EAD=PCD;
		EA=RM16(EAD);
        konami_ICount-=6;
		break;
	case 0x7c:				/* indirect - postbyte offs */
		IMMBYTE(EA);
		EA=PC-1+SIGNED(EA);
		EA=RM16(EAD);
        konami_ICount-=4;
		break;
	case 0x7d:				/* indirect - postword offs */
		IMMWORD(ea);
		EA+=PC-2;
		EA=RM16(EAD);
        konami_ICount-=7;
		break;
	case 0x7e:				/* indirect - normal */
		EAD=PCD;
		EA=RM16(EAD);
        konami_ICount-=3;
		break;
//  case 0x7f: EA=0; break; /* indirect - extended */

//  case 0x80: EA=0; break; /* register a */
//  case 0x81: EA=0; break; /* register b */
//  case 0x82: EA=0; break; /* ???? */
//  case 0x83: EA=0; break; /* ???? */
//  case 0x84: EA=0; break; /* ???? */
//  case 0x85: EA=0; break; /* ???? */
//  case 0x86: EA=0; break; /* ???? */
//  case 0x87: EA=0; break; /* register d */
//  case 0x88: EA=0; break; /* indirect - register a */
//  case 0x89: EA=0; break; /* indirect - register b */
//  case 0x8a: EA=0; break; /* indirect - ???? */
//  case 0x8b: EA=0; break; /* indirect - ???? */
//  case 0x8c: EA=0; break; /* indirect - ???? */
//  case 0x8d: EA=0; break; /* indirect - ???? */
//  case 0x8e: EA=0; break; /* indirect - register d */
//  case 0x8f: EA=0; break; /* indirect - ???? */
//  case 0x90: EA=0; break; /* register a */
//  case 0x91: EA=0; break; /* register b */
//  case 0x92: EA=0; break; /* ???? */
//  case 0x93: EA=0; break; /* ???? */
//  case 0x94: EA=0; break; /* ???? */
//  case 0x95: EA=0; break; /* ???? */
//  case 0x96: EA=0; break; /* ???? */
//  case 0x97: EA=0; break; /* register d */
//  case 0x98: EA=0; break; /* indirect - register a */
//  case 0x99: EA=0; break; /* indirect - register b */
//  case 0x9a: EA=0; break; /* indirect - ???? */
//  case 0x9b: EA=0; break; /* indirect - ???? */
//  case 0x9c: EA=0; break; /* indirect - ???? */
//  case 0x9d: EA=0; break; /* indirect - ???? */
//  case 0x9e: EA=0; break; /* indirect - register d */
//  case 0x9f: EA=0; break; /* indirect - ???? */
	case 0xa0:				/* register a */
		EA=X+SIGNED(A);
        konami_ICount-=1;
		break;
	case 0xa1:				/* register b */
		EA=X+SIGNED(B);
        konami_ICount-=1;
		break;
//  case 0xa2: EA=0; break; /* ???? */
//  case 0xa3: EA=0; break; /* ???? */
//  case 0xa4: EA=0; break; /* ???? */
//  case 0xa5: EA=0; break; /* ???? */
//  case 0xa6: EA=0; break; /* ???? */
	case 0xa7:				/* register d */
		EA=X+D;
        konami_ICount-=4;
		break;
	case 0xa8:				/* indirect - register a */
		EA=X+SIGNED(A);
		EA=RM16(EAD);
        konami_ICount-=4;
		break;
	case 0xa9:				/* indirect - register b */
		EA=X+SIGNED(B);
		EA=RM16(EAD);
        konami_ICount-=4;
		break;
//  case 0xaa: EA=0; break; /* indirect - ???? */
//  case 0xab: EA=0; break; /* indirect - ???? */
//  case 0xac: EA=0; break; /* indirect - ???? */
//  case 0xad: EA=0; break; /* indirect - ???? */
//  case 0xae: EA=0; break; /* indirect - ???? */
	case 0xaf:				/* indirect - register d */
		EA=X+D;
		EA=RM16(EAD);
        konami_ICount-=7;
		break;
	case 0xb0:				/* register a */
		EA=Y+SIGNED(A);
        konami_ICount-=1;
		break;
	case 0xb1:				/* register b */
		EA=Y+SIGNED(B);
        konami_ICount-=1;
		break;
//  case 0xb2: EA=0; break; /* ???? */
//  case 0xb3: EA=0; break; /* ???? */
//  case 0xb4: EA=0; break; /* ???? */
//  case 0xb5: EA=0; break; /* ???? */
//  case 0xb6: EA=0; break; /* ???? */
	case 0xb7:				/* register d */
		EA=Y+D;
        konami_ICount-=4;
		break;
	case 0xb8:				/* indirect - register a */
		EA=Y+SIGNED(A);
		EA=RM16(EAD);
        konami_ICount-=4;
		break;
	case 0xb9:				/* indirect - register b */
		EA=Y+SIGNED(B);
		EA=RM16(EAD);
        konami_ICount-=4;
		break;
//  case 0xba: EA=0; break; /* indirect - ???? */
//  case 0xbb: EA=0; break; /* indirect - ???? */
//  case 0xbc: EA=0; break; /* indirect - ???? */
//  case 0xbd: EA=0; break; /* indirect - ???? */
//  case 0xbe: EA=0; break; /* indirect - ???? */
	case 0xbf:				/* indirect - register d */
		EA=Y+D;
		EA=RM16(EAD);
        konami_ICount-=7;
		break;
//  case 0xc0: EA=0; break; /* register a */
//  case 0xc1: EA=0; break; /* register b */
//  case 0xc2: EA=0; break; /* ???? */
//  case 0xc3: EA=0; break; /* ???? */
	case 0xc4:
		EAD=0;
		(*konami_direct[konami.ireg])();
        konami_ICount -= 1;
		return;
//  case 0xc5: EA=0; break; /* ???? */
//  case 0xc6: EA=0; break; /* ???? */
//  case 0xc7: EA=0; break; /* register d */
//  case 0xc8: EA=0; break; /* indirect - register a */
//  case 0xc9: EA=0; break; /* indirect - register b */
//  case 0xca: EA=0; break; /* indirect - ???? */
//  case 0xcb: EA=0; break; /* indirect - ???? */
	case 0xcc:				/* indirect - direct */
		DIRWORD(ea);
        konami_ICount-=4;
		break;
//  case 0xcd: EA=0; break; /* indirect - ???? */
//  case 0xce: EA=0; break; /* indirect - register d */
//  case 0xcf: EA=0; break; /* indirect - ???? */
	case 0xd0:				/* register a */
		EA=U+SIGNED(A);
        konami_ICount-=1;
		break;
	case 0xd1:				/* register b */
		EA=U+SIGNED(B);
        konami_ICount-=1;
		break;
//  case 0xd2: EA=0; break; /* ???? */
//  case 0xd3: EA=0; break; /* ???? */
//  case 0xd4: EA=0; break; /* ???? */
//  case 0xd5: EA=0; break; /* ???? */
//  case 0xd6: EA=0; break; /* ???? */
	case 0xd7:				/* register d */
		EA=U+D;
        konami_ICount-=4;
		break;
	case 0xd8:				/* indirect - register a */
		EA=U+SIGNED(A);
		EA=RM16(EAD);
        konami_ICount-=4;
		break;
	case 0xd9:				/* indirect - register b */
		EA=U+SIGNED(B);
		EA=RM16(EAD);
        konami_ICount-=4;
		break;
//  case 0xda: EA=0; break; /* indirect - ???? */
//  case 0xdb: EA=0; break; /* indirect - ???? */
//  case 0xdc: EA=0; break; /* indirect - ???? */
//  case 0xdd: EA=0; break; /* indirect - ???? */
//  case 0xde: EA=0; break; /* indirect - ???? */
	case 0xdf:				/* indirect - register d */
		EA=U+D;
		EA=RM16(EAD);
        konami_ICount-=7;
        break;
	case 0xe0:				/* register a */
		EA=S+SIGNED(A);
        konami_ICount-=1;
		break;
	case 0xe1:				/* register b */
		EA=S+SIGNED(B);
        konami_ICount-=1;
		break;
//  case 0xe2: EA=0; break; /* ???? */
//  case 0xe3: EA=0; break; /* ???? */
//  case 0xe4: EA=0; break; /* ???? */
//  case 0xe5: EA=0; break; /* ???? */
//  case 0xe6: EA=0; break; /* ???? */
	case 0xe7:				/* register d */
		EA=S+D;
        konami_ICount-=4;
		break;
	case 0xe8:				/* indirect - register a */
		EA=S+SIGNED(A);
		EA=RM16(EAD);
        konami_ICount-=4;
		break;
	case 0xe9:				/* indirect - register b */
		EA=S+SIGNED(B);
		EA=RM16(EAD);
        konami_ICount-=4;
		break;
//  case 0xea: EA=0; break; /* indirect - ???? */
//  case 0xeb: EA=0; break; /* indirect - ???? */
//  case 0xec: EA=0; break; /* indirect - ???? */
//  case 0xed: EA=0; break; /* indirect - ???? */
//  case 0xee: EA=0; break; /* indirect - ???? */
	case 0xef:				/* indirect - register d */
		EA=S+D;
		EA=RM16(EAD);
        konami_ICount-=7;
		break;
	case 0xf0:				/* register a */
		EA=PC+SIGNED(A);
        konami_ICount-=1;
		break;
	case 0xf1:				/* register b */
		EA=PC+SIGNED(B);
        konami_ICount-=1;
		break;
//  case 0xf2: EA=0; break; /* ???? */
//  case 0xf3: EA=0; break; /* ???? */
//  case 0xf4: EA=0; break; /* ???? */
//  case 0xf5: EA=0; break; /* ???? */
//  case 0xf6: EA=0; break; /* ???? */
	case 0xf7:				/* register d */
		EA=PC+D;
        konami_ICount-=4;
		break;
	case 0xf8:				/* indirect - register a */
		EA=PC+SIGNED(A);
		EA=RM16(EAD);
        konami_ICount-=4;
		break;
	case 0xf9:				/* indirect - register b */
		EA=PC+SIGNED(B);
		EA=RM16(EAD);
        konami_ICount-=4;
		break;
//  case 0xfa: EA=0; break; /* indirect - ???? */
//  case 0xfb: EA=0; break; /* indirect - ???? */
//  case 0xfc: EA=0; break; /* indirect - ???? */
//  case 0xfd: EA=0; break; /* indirect - ???? */
//  case 0xfe: EA=0; break; /* indirect - ???? */
	case 0xff:				/* indirect - register d */
		EA=PC+D;
		EA=RM16(EAD);
        konami_ICount-=7;
		break;
	default:
		logerror("KONAMI: Unknown/Invalid postbyte at PC = %04x\n", PC -1 );
        EAD = 0;
	}
	(*konami_indexed[konami.ireg])();
}


#ifndef __cplusplus
}
#endif

/* execute instructions on this CPU until icount expires */
int konamiRun(int cycles)
{
	konami_ICount = cycles - konami.extra_cycles;
	nCyclesToDo = konami_ICount;
	konami.extra_cycles = 0;

	if( konami.int_state & (KONAMI_CWAI | KONAMI_SYNC) )
	{
		konami_ICount = 0;
	}
	else
	{
		do
		{
			pPPC = pPC;

			konami.ireg = ROP(PCD);
			PC++;

			(*konami_main[konami.ireg])();

			konami_ICount -= cycles1[konami.ireg];

		} while( konami_ICount > 0 );

		konami_ICount -= konami.extra_cycles;
		konami.extra_cycles = 0;
	}

	konami.nTotalCycles += cycles - konami_ICount;
	return cycles - konami_ICount;
}

int konamiCpuScan(int nAction,int */*pnMin*/)
{
	struct BurnArea ba;

	int     (*irq_callback)(int irqline);
	void 	(*setlines_callback)( int lines );

	irq_callback = konami.irq_callback;
	setlines_callback = konami.setlines_callback;

	if (nAction & ACB_DRIVER_DATA) {
		memset(&ba, 0, sizeof(ba));
		ba.Data	  = (unsigned char*)&konami;
		ba.nLen	  = sizeof(konami_Regs);
		ba.szName = "All Registers";
		BurnAcb(&ba);

		SCAN_VAR(ea.w.l);
		SCAN_VAR(ea.d);
	}

	konami.irq_callback = irq_callback;
	konami.setlines_callback = setlines_callback;

	return 0;
}

void konamiSetlinesCallback(void  (*setlines_callback)(int lines))
{
	konami.setlines_callback = setlines_callback;
}

int konamiTotalCycles()
{
	return konami.nTotalCycles;
}

void konamiNewFrame()
{
	konami.nTotalCycles = 0;
}

