/*** hd6309: Portable 6309 emulator ******************************************

    Copyright John Butler
    Copyright Tim Lindner

    References:

        HD63B09EP Technical Refrence Guide, by Chet Simpson with addition
                            by Alan Dekok
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
070614 ZV:
    Fixed N flag setting in DIV overflow

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

000809 TJL:
    Started converting m6809 into hd6309

001217 TJL:
    Finished:
        All opcodes
        Dual Timing
    To Do:
        Verify new DIV opcodes.

070805 TJL:
    Fixed ADDR and ADCR opcodes not to clear the H condition code. Fixed ANDR,
    EORR, ORR, ADDR, ADCR, SBCR, and SUBR to evaluate condition codes after
    the destination register was set. Fixed BITMD opcode to only effect the Z
    condition code. Fixed BITMD opcode to clear only tested flags. Fixed EXG
    and TFR register promotion and demotion. Fixed illegal instruction handler
    to not set I and F condition codes. Credit to Darren Atkinson for the
    discovery of these bugs.

090907 TJL:
    The SEXW instruction is clearing the Overflow flag (V). It should not do
    that. When an invalid source or destination register is specified for
    the TFM instructions, real hardware invokes the Illegal Instruction
    trap, whereas the emulator simply ignores the instruction. Credit to
    Darren Atkinson for the discovery of these bugs.

*****************************************************************************/

//#include "debugger.h"
//#include "deprecat.h"
#include "burnint.h"
#include "hd6309.h"

#define VERBOSE 0

#define LOG(x)	do { if (VERBOSE) logerror x; } while (0)

#ifndef true
#define true 1
#endif

#ifndef false
#define false 0
#endif

#define HD6309_INLINE		static
#define change_pc(newpc)	hd6309.pc.w.l = (newpc)
#define HD6309_CLEAR_LINE	0

//extern offs_t hd6309_dasm(char *buffer, offs_t pc, const UINT8 *oprom, const UINT8 *opram);

/*#define BIG_SWITCH*/

#ifdef __cplusplus
extern "C" {
#endif
static void CHECK_IRQ_LINES( void );
static void IIError(void);
static void DZError(void);

HD6309_INLINE void fetch_effective_address( void );

#ifdef __cplusplus
}
#endif

/* flag bits in the cc register */
#define CC_C	0x01		/* Carry */
#define CC_V	0x02		/* Overflow */
#define CC_Z	0x04		/* Zero */
#define CC_N	0x08		/* Negative */
#define CC_II	0x10		/* Inhibit IRQ */
#define CC_H	0x20		/* Half (auxiliary) carry */
#define CC_IF	0x40		/* Inhibit FIRQ */
#define CC_E	0x80		/* entire state pushed */

/* flag bits in the md register */
#define MD_EM	0x01		/* Execution mode */
#define MD_FM	0x02		/* FIRQ mode */
#define MD_II	0x40		/* Illegal instruction */
#define MD_DZ	0x80		/* Division by zero */

/* 6309 registers */
static hd6309_Regs hd6309;
//static int hd6309_slapstic = 0;

#define pPPC	hd6309.ppc
#define pPC 	hd6309.pc
#define pU		hd6309.u
#define pS		hd6309.s
#define pX		hd6309.x
#define pY		hd6309.y
#define pV		hd6309.v
/*#define pQ        hd6309.q*/
#define pD		hd6309.d
#define pW		hd6309.w
#define pZ		hd6309.z

#define wPPC 	hd6309.ppc.w.l
#define PC		hd6309.pc.w.l
#define PCD 	hd6309.pc.d
#define U		hd6309.u.w.l
#define UD		hd6309.u.d
#define S		hd6309.s.w.l
#define SD		hd6309.s.d
#define X		hd6309.x.w.l
#define XD		hd6309.x.d
#define Y		hd6309.y.w.l
#define YD		hd6309.y.d
#define V		hd6309.v.w.l
#define VD		hd6309.v.d
#define D		hd6309.d.w.l
#define A		hd6309.d.b.h
#define B		hd6309.d.b.l
#define W		hd6309.w.w.l
#define E		hd6309.w.b.h
#define F		hd6309.w.b.l
#define DP		hd6309.dp.b.h
#define DPD 	hd6309.dp.d
#define CC		hd6309.cc
#define MD		hd6309.md

static PAIR ea; 		/* effective address */
#define EA	ea.w.l
#define EAD ea.d

#define CHANGE_PC change_pc(PCD)
#if 0
#define CHANGE_PC	{			\
	if( hd6309_slapstic )		\
		cpu_setOPbase16(PCD);	\
	else						\
		change_pc(PCD);		\
	}
#endif

#define HD6309_CWAI 	8	/* set when CWAI is waiting for an interrupt */
#define HD6309_SYNC 	16	/* set when SYNC is waiting for an interrupt */
#define HD6309_LDS		32	/* set when LDS occured at least once */

/* public globals */
static int hd6309_ICount;

/* these are re-defined in hd6309.h TO RAM, ROM or functions in cpuintrf.c */
#define RM(mAddr)		HD6309_RDMEM(mAddr)
#define WM(mAddr,Value) HD6309_WRMEM(mAddr,Value)
#define ROP(mAddr)		HD6309_RDOP(mAddr)
#define ROP_ARG(mAddr)	HD6309_RDOP_ARG(mAddr)

/* macros to access memory */
#define IMMBYTE(b)	b = ROP_ARG(PCD); PC++
#define IMMWORD(w)	w.d = (ROP_ARG(PCD)<<8) | ROP_ARG((PCD+1)&0xffff); PC+=2
#define IMMLONG(w)	w.d = (ROP_ARG(PCD)<<24) + (ROP_ARG(PCD+1)<<16) + (ROP_ARG(PCD+2)<<8) + (ROP_ARG(PCD+3)); PC+=4

#define PUSHBYTE(b) --S; WM(SD,b)
#define PUSHWORD(w) --S; WM(SD,w.b.l); --S; WM(SD,w.b.h)
#define PULLBYTE(b) b = RM(SD); S++
#define PULLWORD(w) w = RM(SD)<<8; S++; w |= RM(SD); S++

#define PSHUBYTE(b) --U; WM(UD,b);
#define PSHUWORD(w) --U; WM(UD,w.b.l); --U; WM(UD,w.b.h)
#define PULUBYTE(b) b = RM(UD); U++
#define PULUWORD(w) w = RM(UD)<<8; U++; w |= RM(UD); U++

#define CLR_HNZVC	CC&=~(CC_H|CC_N|CC_Z|CC_V|CC_C)
#define CLR_NZV 	CC&=~(CC_N|CC_Z|CC_V)
#define CLR_NZ	 	CC&=~(CC_N|CC_Z)
#define CLR_HNZC	CC&=~(CC_H|CC_N|CC_Z|CC_C)
#define CLR_NZVC	CC&=~(CC_N|CC_Z|CC_V|CC_C)
#define CLR_Z		CC&=~(CC_Z)
#define CLR_N		CC&=~(CC_N)
#define CLR_NZC 	CC&=~(CC_N|CC_Z|CC_C)
#define CLR_ZC		CC&=~(CC_Z|CC_C)

/* macros for CC -- CC bits affected should be reset before calling */
#define SET_Z(a)		if(!a)SEZ
#define SET_Z8(a)		SET_Z((UINT8)a)
#define SET_Z16(a)		SET_Z((UINT16)a)
#define SET_N8(a)		CC|=((a&0x80)>>4)
#define SET_N16(a)		CC|=((a&0x8000)>>12)
#define SET_N32(a)		CC|=((a&0x8000)>>20)
#define SET_H(a,b,r)	CC|=(((a^b^r)&0x10)<<1)
#define SET_C8(a)		CC|=((a&0x100)>>8)
#define SET_C16(a)		CC|=((a&0x10000)>>16)
#define SET_V8(a,b,r)	CC|=(((a^b^r^(r>>1))&0x80)>>6)
#define SET_V16(a,b,r)	CC|=(((a^b^r^(r>>1))&0x8000)>>14)

#define SET_FLAGS8I(a)		{CC|=flags8i[(a)&0xff];}
#define SET_FLAGS8D(a)		{CC|=flags8d[(a)&0xff];}

static UINT8 const *cycle_counts_page0;
static UINT8 const *cycle_counts_page01;
static UINT8 const *cycle_counts_page11;
static UINT8 const *index_cycle;

/* combos */
#define SET_NZ8(a)			{SET_N8(a);SET_Z(a);}
#define SET_NZ16(a) 		{SET_N16(a);SET_Z(a);}
#define SET_FLAGS8(a,b,r)	{SET_N8(r);SET_Z8(r);SET_V8(a,b,r);SET_C8(r);}
#define SET_FLAGS16(a,b,r)	{SET_N16(r);SET_Z16(r);SET_V16(a,b,r);SET_C16(r);}

#define NXORV				((CC&CC_N)^((CC&CC_V)<<2))

/* for treating an unsigned byte as a signed word */
#define SIGNED(b) ((UINT16)(b&0x80?b|0xff00:b))
/* for treating an unsigned short as a signed long */
#define SIGNED_16(b) ((UINT32)(b&0x8000?b|0xffff0000:b))

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

/* Macros to set mode flags */
#define SEDZ MD|=MD_DZ
#define CLDZ MD&=~MD_DZ
#define SEII MD|=MD_II
#define CLII MD&=~MD_II
#define SEFM MD|=MD_FM
#define CLFM MD&=~MD_FM
#define SEEM MD|=MD_EM
#define CLEM MD&=~MD_EM

/* macros for convenience */
#define DIRBYTE(b) {DIRECT;b=RM(EAD);}
#define DIRWORD(w) {DIRECT;w.d=RM16(EAD);}
#define DIRLONG(lng) {DIRECT;lng.w.h=RM16(EAD);lng.w.l=RM16(EAD+2);}
#define EXTBYTE(b) {EXTENDED;b=RM(EAD);}
#define EXTWORD(w) {EXTENDED;w.d=RM16(EAD);}
#define EXTLONG(lng) {EXTENDED;lng.w.h=RM16(EAD);lng.w.l=RM16(EAD+2);}

/* includes the static function prototypes and other tables */
#ifdef __cplusplus
extern "C" {
#endif

HD6309_INLINE void illegal( void );
HD6309_INLINE void neg_di( void );
HD6309_INLINE void oim_di( void );
HD6309_INLINE void aim_di( void );
HD6309_INLINE void com_di( void );
HD6309_INLINE void lsr_di( void );
HD6309_INLINE void eim_di( void );
HD6309_INLINE void ror_di( void );
HD6309_INLINE void asr_di( void );
HD6309_INLINE void asl_di( void );
HD6309_INLINE void rol_di( void );
HD6309_INLINE void dec_di( void );
HD6309_INLINE void tim_di( void );
HD6309_INLINE void inc_di( void );
HD6309_INLINE void tst_di( void );
HD6309_INLINE void jmp_di( void );
HD6309_INLINE void clr_di( void );
HD6309_INLINE void nop( void );
HD6309_INLINE void sync( void );
HD6309_INLINE void sexw( void );
HD6309_INLINE void lbra( void );
HD6309_INLINE void lbsr( void );
HD6309_INLINE void daa( void );
HD6309_INLINE void daa( void );
HD6309_INLINE void orcc( void );
HD6309_INLINE void andcc( void );
HD6309_INLINE void sex( void );
HD6309_INLINE void exg( void );
HD6309_INLINE void tfr( void );
HD6309_INLINE void bra( void );
HD6309_INLINE void brn( void );
HD6309_INLINE void lbrn( void );
HD6309_INLINE void bhi( void );
HD6309_INLINE void lbhi( void );
HD6309_INLINE void bls( void );
HD6309_INLINE void lbls( void );
HD6309_INLINE void bcc( void );
HD6309_INLINE void lbcc( void );
HD6309_INLINE void bcs( void );
HD6309_INLINE void lbcs( void );
HD6309_INLINE void bne( void );
HD6309_INLINE void lbne( void );
HD6309_INLINE void beq( void );
HD6309_INLINE void lbeq( void );
HD6309_INLINE void bvc( void );
HD6309_INLINE void lbvc( void );
HD6309_INLINE void bvs( void );
HD6309_INLINE void lbvs( void );
HD6309_INLINE void bpl( void );
HD6309_INLINE void lbpl( void );
HD6309_INLINE void bmi( void );
HD6309_INLINE void lbmi( void );
HD6309_INLINE void bge( void );
HD6309_INLINE void lbge( void );
HD6309_INLINE void blt( void );
HD6309_INLINE void lblt( void );
HD6309_INLINE void bgt( void );
HD6309_INLINE void lbgt( void );
HD6309_INLINE void ble( void );
HD6309_INLINE void lble( void );
HD6309_INLINE void addr_r( void );
HD6309_INLINE void adcr( void );
HD6309_INLINE void subr( void );
HD6309_INLINE void sbcr( void );
HD6309_INLINE void andr( void );
HD6309_INLINE void orr( void );
HD6309_INLINE void eorr( void );
HD6309_INLINE void cmpr( void );
HD6309_INLINE void tfmpp( void );
HD6309_INLINE void tfmmm( void );
HD6309_INLINE void tfmpc( void );
HD6309_INLINE void tfmcp( void );
HD6309_INLINE void bitmd_im( void );
HD6309_INLINE void leax( void );
HD6309_INLINE void leay( void );
HD6309_INLINE void leas( void );
HD6309_INLINE void leau( void );
HD6309_INLINE void pshs( void );
HD6309_INLINE void ldmd_im( void );
HD6309_INLINE void pshsw( void );
HD6309_INLINE void pshuw( void );
HD6309_INLINE void puls( void );
HD6309_INLINE void pulsw( void );
HD6309_INLINE void puluw( void );
HD6309_INLINE void pshu( void );
HD6309_INLINE void pulu( void );
HD6309_INLINE void rts( void );
HD6309_INLINE void abx( void );
HD6309_INLINE void rti( void );
HD6309_INLINE void cwai( void );
HD6309_INLINE void bitd_di( void );
HD6309_INLINE void bitd_ix( void );
HD6309_INLINE void bitd_ex( void );
HD6309_INLINE void mul( void );
HD6309_INLINE void swi( void );
HD6309_INLINE void band( void );
HD6309_INLINE void bitd_im( void );
HD6309_INLINE void biand( void );
HD6309_INLINE void bor( void );
HD6309_INLINE void bior( void );
HD6309_INLINE void beor( void );
HD6309_INLINE void bieor( void );
HD6309_INLINE void ldbt( void );
HD6309_INLINE void stbt( void );
HD6309_INLINE void swi2( void );
HD6309_INLINE void swi3( void );
HD6309_INLINE void nega( void );
HD6309_INLINE void coma( void );
HD6309_INLINE void lsra( void );
HD6309_INLINE void rora( void );
HD6309_INLINE void asra( void );
HD6309_INLINE void asla( void );
HD6309_INLINE void rola( void );
HD6309_INLINE void deca( void );
HD6309_INLINE void inca( void );
HD6309_INLINE void tsta( void );
HD6309_INLINE void clra( void );
HD6309_INLINE void negb( void );
HD6309_INLINE void negd( void );
HD6309_INLINE void comb( void );
HD6309_INLINE void come( void );
HD6309_INLINE void comf( void );
HD6309_INLINE void comd( void );
HD6309_INLINE void comw( void );
HD6309_INLINE void lsrb( void );
HD6309_INLINE void lsrd( void );
HD6309_INLINE void lsrw( void );
HD6309_INLINE void rorb( void );
HD6309_INLINE void rord( void );
HD6309_INLINE void rorw( void );
HD6309_INLINE void asrb( void );
HD6309_INLINE void asrd( void );
HD6309_INLINE void aslb( void );
HD6309_INLINE void asld( void );
HD6309_INLINE void rolb( void );
HD6309_INLINE void rold( void );
HD6309_INLINE void rolw( void );
HD6309_INLINE void decb( void );
HD6309_INLINE void dece( void );
HD6309_INLINE void decf( void );
HD6309_INLINE void decd( void );
HD6309_INLINE void decw( void );
HD6309_INLINE void incb( void );
HD6309_INLINE void ince( void );
HD6309_INLINE void incf( void );
HD6309_INLINE void incd( void );
HD6309_INLINE void incw( void );
HD6309_INLINE void tstb( void );
HD6309_INLINE void tstd( void );
HD6309_INLINE void tstw( void );
HD6309_INLINE void tste( void );
HD6309_INLINE void tstf( void );
HD6309_INLINE void clrb( void );
HD6309_INLINE void clrd( void );
HD6309_INLINE void clre( void );
HD6309_INLINE void clrf( void );
HD6309_INLINE void clrw( void );
HD6309_INLINE void neg_ix( void );
HD6309_INLINE void oim_ix( void );
HD6309_INLINE void aim_ix( void );
HD6309_INLINE void com_ix( void );
HD6309_INLINE void lsr_ix( void );
HD6309_INLINE void eim_ix( void );
HD6309_INLINE void ror_ix( void );
HD6309_INLINE void asr_ix( void );
HD6309_INLINE void asl_ix( void );
HD6309_INLINE void rol_ix( void );
HD6309_INLINE void dec_ix( void );
HD6309_INLINE void tim_ix( void );
HD6309_INLINE void inc_ix( void );
HD6309_INLINE void tst_ix( void );
HD6309_INLINE void jmp_ix( void );
HD6309_INLINE void clr_ix( void );
HD6309_INLINE void neg_ex( void );
HD6309_INLINE void oim_ex( void );
HD6309_INLINE void aim_ex( void );
HD6309_INLINE void com_ex( void );
HD6309_INLINE void lsr_ex( void );
HD6309_INLINE void eim_ex( void );
HD6309_INLINE void ror_ex( void );
HD6309_INLINE void asr_ex( void );
HD6309_INLINE void asl_ex( void );
HD6309_INLINE void rol_ex( void );
HD6309_INLINE void dec_ex( void );
HD6309_INLINE void tim_ex( void );
HD6309_INLINE void inc_ex( void );
HD6309_INLINE void tst_ex( void );
HD6309_INLINE void jmp_ex( void );
HD6309_INLINE void clr_ex( void );
HD6309_INLINE void suba_im( void );
HD6309_INLINE void cmpa_im( void );
HD6309_INLINE void sbca_im( void );
HD6309_INLINE void subd_im( void );
HD6309_INLINE void subw_im( void );
HD6309_INLINE void cmpd_im( void );
HD6309_INLINE void cmpw_im( void );
HD6309_INLINE void cmpu_im( void );
HD6309_INLINE void anda_im( void );
HD6309_INLINE void bita_im( void );
HD6309_INLINE void lda_im( void );
HD6309_INLINE void eora_im( void );
HD6309_INLINE void adca_im( void );
HD6309_INLINE void ora_im( void );
HD6309_INLINE void adda_im( void );
HD6309_INLINE void cmpx_im( void );
HD6309_INLINE void cmpy_im( void );
HD6309_INLINE void cmps_im( void );
HD6309_INLINE void bsr( void );
HD6309_INLINE void ldx_im( void );
HD6309_INLINE void ldq_im( void );
HD6309_INLINE void ldy_im( void );
HD6309_INLINE void suba_di( void );
HD6309_INLINE void cmpa_di( void );
HD6309_INLINE void sbca_di( void );
HD6309_INLINE void subd_di( void );
HD6309_INLINE void subw_di( void );
HD6309_INLINE void cmpd_di( void );
HD6309_INLINE void cmpw_di( void );
HD6309_INLINE void cmpu_di( void );
HD6309_INLINE void anda_di( void );
HD6309_INLINE void bita_di( void );
HD6309_INLINE void lda_di( void );
HD6309_INLINE void sta_di( void );
HD6309_INLINE void eora_di( void );
HD6309_INLINE void adca_di( void );
HD6309_INLINE void ora_di( void );
HD6309_INLINE void adda_di( void );
HD6309_INLINE void cmpx_di( void );
HD6309_INLINE void cmpy_di( void );
HD6309_INLINE void cmps_di( void );
HD6309_INLINE void jsr_di( void );
HD6309_INLINE void ldx_di( void );
HD6309_INLINE void muld_di( void );
HD6309_INLINE void divd_im( void );
HD6309_INLINE void divq_im( void );
HD6309_INLINE void muld_im( void );
HD6309_INLINE void divd_di( void );
HD6309_INLINE void divq_di( void );
HD6309_INLINE void ldq_di( void );
HD6309_INLINE void ldy_di( void );
HD6309_INLINE void stx_di( void );
HD6309_INLINE void stq_di( void );
HD6309_INLINE void sty_di( void );
HD6309_INLINE void suba_ix( void );
HD6309_INLINE void cmpa_ix( void );
HD6309_INLINE void sbca_ix( void );
HD6309_INLINE void subd_ix( void );
HD6309_INLINE void subw_ix( void );
HD6309_INLINE void cmpd_ix( void );
HD6309_INLINE void cmpw_ix( void );
HD6309_INLINE void cmpu_ix( void );
HD6309_INLINE void anda_ix( void );
HD6309_INLINE void bita_ix( void );
HD6309_INLINE void lda_ix( void );
HD6309_INLINE void sta_ix( void );
HD6309_INLINE void eora_ix( void );
HD6309_INLINE void adca_ix( void );
HD6309_INLINE void ora_ix( void );
HD6309_INLINE void adda_ix( void );
HD6309_INLINE void cmpx_ix( void );
HD6309_INLINE void cmpy_ix( void );
HD6309_INLINE void cmps_ix( void );
HD6309_INLINE void jsr_ix( void );
HD6309_INLINE void ldx_ix( void );
HD6309_INLINE void muld_ix( void );
HD6309_INLINE void divd_ix( void );
HD6309_INLINE void divq_ix( void );
HD6309_INLINE void ldq_ix( void );
HD6309_INLINE void ldy_ix( void );
HD6309_INLINE void stx_ix( void );
HD6309_INLINE void stq_ix( void );
HD6309_INLINE void sty_ix( void );
HD6309_INLINE void suba_ex( void );
HD6309_INLINE void cmpa_ex( void );
HD6309_INLINE void sbca_ex( void );
HD6309_INLINE void subd_ex( void );
HD6309_INLINE void subw_ex( void );
HD6309_INLINE void cmpd_ex( void );
HD6309_INLINE void cmpw_ex( void );
HD6309_INLINE void cmpu_ex( void );
HD6309_INLINE void anda_ex( void );
HD6309_INLINE void bita_ex( void );
HD6309_INLINE void lda_ex( void );
HD6309_INLINE void sta_ex( void );
HD6309_INLINE void eora_ex( void );
HD6309_INLINE void adca_ex( void );
HD6309_INLINE void ora_ex( void );
HD6309_INLINE void adda_ex( void );
HD6309_INLINE void cmpx_ex( void );
HD6309_INLINE void cmpy_ex( void );
HD6309_INLINE void cmps_ex( void );
HD6309_INLINE void jsr_ex( void );
HD6309_INLINE void ldx_ex( void );
HD6309_INLINE void muld_ex( void );
HD6309_INLINE void divd_ex( void );
HD6309_INLINE void divq_ex( void );
HD6309_INLINE void ldq_ex( void );
HD6309_INLINE void ldy_ex( void );
HD6309_INLINE void stx_ex( void );
HD6309_INLINE void stq_ex( void );
HD6309_INLINE void sty_ex( void );
HD6309_INLINE void subb_im( void );
HD6309_INLINE void sube_im( void );
HD6309_INLINE void subf_im( void );
HD6309_INLINE void cmpb_im( void );
HD6309_INLINE void cmpe_im( void );
HD6309_INLINE void cmpf_im( void );
HD6309_INLINE void sbcb_im( void );
HD6309_INLINE void sbcd_im( void );
HD6309_INLINE void addd_im( void );
HD6309_INLINE void addw_im( void );
HD6309_INLINE void adde_im( void );
HD6309_INLINE void addf_im( void );
HD6309_INLINE void andb_im( void );
HD6309_INLINE void andd_im( void );
HD6309_INLINE void bitb_im( void );
HD6309_INLINE void ldb_im( void );
HD6309_INLINE void lde_im( void );
HD6309_INLINE void ldf_im( void );
HD6309_INLINE void eorb_im( void );
HD6309_INLINE void eord_im( void );
HD6309_INLINE void adcb_im( void );
HD6309_INLINE void adcd_im( void );
HD6309_INLINE void orb_im( void );
HD6309_INLINE void ord_im( void );
HD6309_INLINE void addb_im( void );
HD6309_INLINE void ldd_im( void );
HD6309_INLINE void ldw_im( void );
HD6309_INLINE void ldu_im( void );
HD6309_INLINE void lds_im( void );
HD6309_INLINE void subb_di( void );
HD6309_INLINE void sube_di( void );
HD6309_INLINE void subf_di( void );
HD6309_INLINE void cmpb_di( void );
HD6309_INLINE void cmpe_di( void );
HD6309_INLINE void cmpf_di( void );
HD6309_INLINE void sbcb_di( void );
HD6309_INLINE void sbcd_di( void );
HD6309_INLINE void addd_di( void );
HD6309_INLINE void addw_di( void );
HD6309_INLINE void adde_di( void );
HD6309_INLINE void addf_di( void );
HD6309_INLINE void andb_di( void );
HD6309_INLINE void andd_di( void );
HD6309_INLINE void bitb_di( void );
HD6309_INLINE void ldb_di( void );
HD6309_INLINE void lde_di( void );
HD6309_INLINE void ldf_di( void );
HD6309_INLINE void stb_di( void );
HD6309_INLINE void ste_di( void );
HD6309_INLINE void stf_di( void );
HD6309_INLINE void eorb_di( void );
HD6309_INLINE void eord_di( void );
HD6309_INLINE void adcb_di( void );
HD6309_INLINE void adcd_di( void );
HD6309_INLINE void orb_di( void );
HD6309_INLINE void ord_di( void );
HD6309_INLINE void addb_di( void );
HD6309_INLINE void ldd_di( void );
HD6309_INLINE void ldw_di( void );
HD6309_INLINE void std_di( void );
HD6309_INLINE void stw_di( void );
HD6309_INLINE void ldu_di( void );
HD6309_INLINE void lds_di( void );
HD6309_INLINE void stu_di( void );
HD6309_INLINE void sts_di( void );
HD6309_INLINE void subb_ix( void );
HD6309_INLINE void sube_ix( void );
HD6309_INLINE void subf_ix( void );
HD6309_INLINE void cmpb_ix( void );
HD6309_INLINE void cmpe_ix( void );
HD6309_INLINE void cmpf_ix( void );
HD6309_INLINE void sbcb_ix( void );
HD6309_INLINE void sbcd_ix( void );
HD6309_INLINE void addd_ix( void );
HD6309_INLINE void addw_ix( void );
HD6309_INLINE void adde_ix( void );
HD6309_INLINE void addf_ix( void );
HD6309_INLINE void andb_ix( void );
HD6309_INLINE void andd_ix( void );
HD6309_INLINE void bitb_ix( void );
HD6309_INLINE void ldb_ix( void );
HD6309_INLINE void lde_ix( void );
HD6309_INLINE void ldf_ix( void );
HD6309_INLINE void stb_ix( void );
HD6309_INLINE void ste_ix( void );
HD6309_INLINE void stf_ix( void );
HD6309_INLINE void eorb_ix( void );
HD6309_INLINE void eord_ix( void );
HD6309_INLINE void adcb_ix( void );
HD6309_INLINE void adcd_ix( void );
HD6309_INLINE void orb_ix( void );
HD6309_INLINE void ord_ix( void );
HD6309_INLINE void addb_ix( void );
HD6309_INLINE void ldd_ix( void );
HD6309_INLINE void ldw_ix( void );
HD6309_INLINE void std_ix( void );
HD6309_INLINE void stw_ix( void );
HD6309_INLINE void ldu_ix( void );
HD6309_INLINE void lds_ix( void );
HD6309_INLINE void stu_ix( void );
HD6309_INLINE void sts_ix( void );
HD6309_INLINE void subb_ex( void );
HD6309_INLINE void sube_ex( void );
HD6309_INLINE void subf_ex( void );
HD6309_INLINE void cmpb_ex( void );
HD6309_INLINE void cmpe_ex( void );
HD6309_INLINE void cmpf_ex( void );
HD6309_INLINE void sbcb_ex( void );
HD6309_INLINE void sbcd_ex( void );
HD6309_INLINE void addd_ex( void );
HD6309_INLINE void addw_ex( void );
HD6309_INLINE void adde_ex( void );
HD6309_INLINE void addf_ex( void );
HD6309_INLINE void andb_ex( void );
HD6309_INLINE void andd_ex( void );
HD6309_INLINE void bitb_ex( void );
HD6309_INLINE void ldb_ex( void );
HD6309_INLINE void lde_ex( void );
HD6309_INLINE void ldf_ex( void );
HD6309_INLINE void stb_ex( void );
HD6309_INLINE void ste_ex( void );
HD6309_INLINE void stf_ex( void );
HD6309_INLINE void eorb_ex( void );
HD6309_INLINE void eord_ex( void );
HD6309_INLINE void adcb_ex( void );
HD6309_INLINE void adcd_ex( void );
HD6309_INLINE void orb_ex( void );
HD6309_INLINE void ord_ex( void );
HD6309_INLINE void addb_ex( void );
HD6309_INLINE void ldd_ex( void );
HD6309_INLINE void ldw_ex( void );
HD6309_INLINE void std_ex( void );
HD6309_INLINE void stw_ex( void );
HD6309_INLINE void ldu_ex( void );
HD6309_INLINE void lds_ex( void );
HD6309_INLINE void stu_ex( void );
HD6309_INLINE void sts_ex( void );
HD6309_INLINE void pref10( void );
HD6309_INLINE void pref11( void );

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

static const UINT8 index_cycle_em[256] = {        /* Index Loopup cycle counts, emulated 6809 */
/*           0xX0, 0xX1, 0xX2, 0xX3, 0xX4, 0xX5, 0xX6, 0xX7, 0xX8, 0xX9, 0xXA, 0xXB, 0xXC, 0xXD, 0xXE, 0xXF */

/* 0x0X */      1,    1,    1,    1,    1,    1,    1,    1,    1,    1,    1,    1,    1,    1,    1,    1,
/* 0x1X */      1,    1,    1,    1,    1,    1,    1,    1,    1,    1,    1,    1,    1,    1,    1,    1,
/* 0x2X */      1,    1,    1,    1,    1,    1,    1,    1,    1,    1,    1,    1,    1,    1,    1,    1,
/* 0x3X */      1,    1,    1,    1,    1,    1,    1,    1,    1,    1,    1,    1,    1,    1,    1,    1,
/* 0x4X */      1,    1,    1,    1,    1,    1,    1,    1,    1,    1,    1,    1,    1,    1,    1,    1,
/* 0x5X */      1,    1,    1,    1,    1,    1,    1,    1,    1,    1,    1,    1,    1,    1,    1,    1,
/* 0x6X */      1,    1,    1,    1,    1,    1,    1,    1,    1,    1,    1,    1,    1,    1,    1,    1,
/* 0x7X */      1,    1,    1,    1,    1,    1,    1,    1,    1,    1,    1,    1,    1,    1,    1,    1,
/* 0x8X */      2,    3,    2,    3,    0,    1,    1,    1,    1,    4,    1,    4,    1,    5,    4,    0,
/* 0x9X */      3,    6,   20,    6,    3,    4,    4,    4,    4,    7,    4,    7,    4,    8,    7,    5,
/* 0xAX */      2,    3,    2,    3,    0,    1,    1,    1,    1,    4,    1,    4,    1,    5,    4,    5,
/* 0xBX */      5,    6,   20,    6,    3,    4,    4,    4,    4,    7,    4,    7,    4,    8,    7,   20,
/* 0xCX */      2,    3,    2,    3,    0,    1,    1,    1,    1,    4,    1,    4,    1,    5,    4,    3,
/* 0xDX */      4,    6,   20,    6,    3,    4,    4,    4,    4,    7,    4,    7,    4,    8,    7,   20,
/* 0xEX */      2,    3,    2,    3,    0,    1,    1,    1,    1,    4,    1,    4,    1,    5,    4,    3,
/* 0xFX */      4,    6,   20,    6,    3,    4,    4,    4,    4,    7,    4,    7,    4,    8,    7,   20
};

static const UINT8 index_cycle_na[256] = {         /* Index Loopup cycle counts,
native 6309 */
/*       X0, X1, X2, X3, X4, X5, X6, X7, X8, X9, XA, XB, XC, XD, XE, XF */

/* 0x0X */   1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,
/* 0x1X */   1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,
/* 0x2X */   1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,
/* 0x3X */   1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,
/* 0x4X */   1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,
/* 0x5X */   1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,
/* 0x6X */   1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,
/* 0x7X */   1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,
/* 0x8X */   1,  2,  1,  2,  0,  1,  1,  1,  1,  3,  1,  2,  1,  3,  1,  0,
/* 0x9X */   3,  5, 19,  5,  3,  4,  4,  4,  4,  7,  4,  5,  4,  6,  4,  5,
/* 0xAX */   1,  2,  1,  2,  0,  1,  1,  1,  1,  3,  1,  2,  1,  3,  1,  2,
/* 0xBX */   5,  5, 19,  5,  3,  4,  4,  4,  4,  7,  4,  5,  4,  6,  4, 19,
/* 0xCX */   1,  2,  1,  2,  0,  1,  1,  1,  1,  3,  1,  2,  1,  3,  1,  1,
/* 0xDX */   4,  5, 19,  5,  3,  4,  4,  4,  4,  7,  4,  5,  4,  6,  4, 19,
/* 0xEX */   1,  2,  1,  2,  0,  1,  1,  1,  1,  3,  1,  2,  1,  3,  1,  1,
/* 0xFX */   4,  5, 19,  5,  3,  4,  4,  4,  4,  7,  4,  5,  4,  6,  4, 19
};

#define IIP0	19			/* Illegal instruction cycle count page 0 */
#define IIP1	20			/* Illegal instruction cycle count page 01 & 11 */

static const UINT8 ccounts_page0_em[256] =    /* Cycle Counts Page zero, Emulated 6809 */
{
/*           0xX0, 0xX1, 0xX2, 0xX3, 0xX4, 0xX5, 0xX6, 0xX7, 0xX8, 0xX9, 0xXA, 0xXB, 0xXC, 0xXD, 0xXE, 0xXF */
/* 0x0X */     6,    6,    6,    6,    6,    6,    6,    6,    6,    6,    6,    6,    6,    6,    3,    6,
/* 0x1X */     0,    0,    2,    4,    4, IIP0,    5,    9, IIP0,    2,    3, IIP0,    3,    2,    8,    6,
/* 0x2X */     3,    3,    3,    3,    3,    3,    3,    3,    3,    3,    3,    3,    3,    3,    3,    3,
/* 0x3X */     4,    4,    4,    4,    5,    5,    5,    5, IIP0,    5,    3,    6,   20,   11, IIP0,   19,
/* 0x4X */     2, IIP0, IIP0,    2,    2, IIP0,    2,    2,    2,    2,    2, IIP0,    2,    2, IIP0,    2,
/* 0x5X */     2, IIP0, IIP0,    2,    2, IIP0,    2,    2,    2,    2,    2, IIP0,    2,    2, IIP0,    2,
/* 0x6X */     6,    7,    7,    6,    6,    6,    6,    6,    6,    6,    6,    7,    6,    6,    3,    6,
/* 0x7X */     7,    7,    7,    7,    7,    7,    7,    7,    7,    7,    7,    5,    7,    7,    4,    7,
/* 0x8X */     2,    2,    2,    4,    2,    2,    2, IIP0,    2,    2,    2,    2,    4,    7,    3, IIP0,
/* 0x9X */     4,    4,    4,    6,    4,    4,    4,    4,    4,    4,    4,    4,    6,    7,    5,    5,
/* 0xAX */     4,    4,    4,    6,    4,    4,    4,    4,    4,    4,    4,    4,    6,    7,    5,    5,
/* 0xBX */     5,    5,    5,    7,    5,    5,    5,    5,    5,    5,    5,    5,    7,    8,    6,    6,
/* 0xCX */     2,    2,    2,    4,    2,    2,    2, IIP0,    2,    2,    2,    2,    3,    5,    3, IIP0,
/* 0xDX */     4,    4,    4,    6,    4,    4,    4,    4,    4,    4,    4,    4,    5,    5,    5,    5,
/* 0xEX */     4,    4,    4,    6,    4,    4,    4,    4,    4,    4,    4,    4,    5,    5,    5,    5,
/* 0xFX */     5,    5,    5,    7,    5,    5,    5,    5,    5,    5,    5,    5,    6,    6,    6,    6
};

static const UINT8 ccounts_page0_na[256] =   /* Cycle Counts Page zero, Native 6309 */
{
/*           0xX0, 0xX1, 0xX2, 0xX3, 0xX4, 0xX5, 0xX6, 0xX7, 0xX8, 0xX9, 0xXA, 0xXB, 0xXC, 0xXD, 0xXE, 0xXF */
/* 0x0X */     5,    6,    6,    5,    5,    6,    5,    5,    5,    5,    5,    6,    5,    4,    2,    5,
/* 0x1X */     0,    0,    1,    4,    4, IIP0,    4,    7, IIP0,    1,    2, IIP0,    3,    1,    5,    4,
/* 0x2X */     3,    3,    3,    3,    3,    3,    3,    3,    3,    3,    3,    3,    3,    3,    3,    3,
/* 0x3X */     4,    4,    4,    4,    4,    4,    4,    4, IIP0,    4,    1,    6,   22,   10, IIP0,   21,
/* 0x4X */     1, IIP0, IIP0,    1,    1, IIP0,    1,    1,    1,    1,    1, IIP0,    1,    1, IIP0,    1,
/* 0x5X */     1, IIP0, IIP0,    1,    1, IIP0,    1,    1,    1,    1,    1, IIP0,    1,    1, IIP0,    1,
/* 0x6X */     6,    7,    7,    6,    6,    6,    6,    6,    6,    6,    6,    7,    6,    5,    3,    6,
/* 0x7X */     6,    7,    7,    6,    6,    7,    6,    6,    6,    6,    6,    5,    6,    5,    3,    6,
/* 0x8X */     2,    2,    2,    3,    2,    2,    2, IIP0,    2,    2,    2,    2,    3,    6,    3, IIP0,
/* 0x9X */     3,    3,    3,    4,    3,    3,    3,    3,    3,    3,    3,    3,    4,    6,    4,    4,
/* 0xAX */     4,    4,    4,    5,    4,    4,    4,    4,    4,    4,    4,    4,    5,    6,    5,    5,
/* 0xBX */     4,    4,    4,    5,    4,    4,    4,    4,    4,    4,    4,    4,    5,    7,    5,    5,
/* 0xCX */     2,    2,    2,    3,    2,    2,    2, IIP0,    2,    2,    2,    2,    3,    5,    3, IIP0,
/* 0xDX */     3,    3,    3,    4,    3,    3,    3,    3,    3,    3,    3,    3,    4,    4,    4,    4,
/* 0xEX */     4,    4,    4,    5,    4,    4,    4,    4,    4,    4,    4,    4,    5,    5,    5,    5,
/* 0xFX */     4,    4,    4,    5,    4,    4,    4,    4,    4,    4,    4,    4,    5,    5,    5,    5
};

static const UINT8 ccounts_page01_em[256] =    /* Cycle Counts Page 01, Emulated 6809 */
{
/*           0xX0, 0xX1, 0xX2, 0xX3, 0xX4, 0xX5, 0xX6, 0xX7, 0xX8, 0xX9, 0xXA, 0xXB, 0xXC, 0xXD, 0xXE, 0xXF */
/* 0x0X */   IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1,
/* 0x1X */   IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1,
/* 0x2X */   IIP1,    5,    5,    5,    5,    5,    5,    5,    5,    5,    5,    5,    5,    5,    5,    5,
/* 0x3X */      4,    4,    4,    4,    4,    4,    4,    4,    6,    6,    6,    6, IIP1, IIP1, IIP1,   20,
/* 0x4X */      2,  IIP1,IIP1,    2,    2, IIP1,    2,    2,    2,    2,    2, IIP1,    2,    2, IIP1,    2,
/* 0x5X */   IIP1, IIP1, IIP1,    3,    3, IIP1,    3, IIP1, IIP1,    3,    3, IIP1,    3,    3, IIP1,    3,
/* 0x6X */   IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1,
/* 0x7X */   IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1,
/* 0x8X */      5,    5,    5,    5,    5,    5,    4, IIP1,    5,    5,    5,    5,    5, IIP1,    4, IIP1,
/* 0x9X */      7,    7,    7,    7,    7,    7,    6,    6,    7,    7,    7,    7,    7, IIP1,    6,    6,
/* 0xAX */      7,    7,    7,    7,    7,    7,    6,    6,    7,    7,    7,    7,    7, IIP1,    6,    6,
/* 0xBX */      8,    8,    8,    8,    8,    8,    7,    7,    8,    8,    8,    8,    8, IIP1,    7,    7,
/* 0xCX */   IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1,    4, IIP1,
/* 0xDX */   IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1,    8,    8,    6,    6,
/* 0xEX */   IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1,    8,    8,    6,    6,
/* 0xFX */   IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1,    9,    9,    7,    7
};

static const UINT8 ccounts_page01_na[256] =   /* Cycle Counts Page 01, Native 6309 */
{
/*           0xX0, 0xX1, 0xX2, 0xX3, 0xX4, 0xX5, 0xX6, 0xX7, 0xX8, 0xX9, 0xXA, 0xXB, 0xXC, 0xXD, 0xXE, 0xXF */
/* 0x0X */   IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1,
/* 0x1X */   IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1,
/* 0x2X */   IIP1,    5,    5,    5,    5,    5,    5,    5,    5,    5,    5,    5,    5,    5,    5,    5,
/* 0x3X */      4,    4,    4,    4,    4,    4,    4,    4,    6,    6,    6,    6, IIP1, IIP1, IIP1,   22,
/* 0x4X */      1, IIP1, IIP1,    1,    1, IIP1,    1,    1,    1,    1,    1, IIP1,    1,    1, IIP1,    1,
/* 0x5X */   IIP1, IIP1, IIP1,    2,    2, IIP1,    2, IIP1, IIP1,    2,    2, IIP1,    2,    2, IIP1,    1,
/* 0x6X */   IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1,
/* 0x7X */   IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1,
/* 0x8X */      4,    4,    4,    4,    4,    4,    4, IIP1,    4,    4,    4,    4,    4, IIP1,    4, IIP1,
/* 0x9X */      5,    5,    5,    5,    5,    5,    5,    5,    5,    5,    5,    5,    5, IIP1,    5,    5,
/* 0xAX */      6,    6,    6,    6,    6,    6,    6,    6,    6,    6,    6,    6,    6, IIP1,    6,    6,
/* 0xBX */      6,    6,    6,    6,    6,    6,    6,    6,    6,    6,    6,    6,    6, IIP1,    6,    6,
/* 0xCX */   IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1,    4, IIP1,
/* 0xDX */   IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1,    7,    7,    5,    5,
/* 0xEX */   IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1,    8,    8,    6,    6,
/* 0xFX */   IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1,    8,    8,    6,    6
};

static const UINT8 ccounts_page11_em[256] =    /* Cycle Counts Page 11, Emulated 6809 */
{
/*           0xX0, 0xX1, 0xX2, 0xX3, 0xX4, 0xX5, 0xX6, 0xX7, 0xX8, 0xX9, 0xXA, 0xXB, 0xXC, 0xXD, 0xXE, 0xXF */
/* 0x0X */   IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1,
/* 0x1X */   IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1,
/* 0x2X */   IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1,
/* 0x3X */      7,    7,    7,    7,    7,    7,    7,    8,    3,    3,    3,    3,    4,    5, IIP1,   20,
/* 0x4X */   IIP1, IIP1, IIP1,    2, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1,    2, IIP1,    2,    2, IIP1,    2,
/* 0x5X */   IIP1, IIP1, IIP1,    2, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1,    2, IIP1,    2,    2, IIP1,    2,
/* 0x6X */   IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1,
/* 0x7X */   IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1,
/* 0x8X */      3,    3, IIP1,    5, IIP1, IIP1,    3, IIP1, IIP1, IIP1, IIP1,    3,    5,   25,   34,   28,
/* 0x9X */      5,    5, IIP1,    7, IIP1, IIP1,    5,    5, IIP1, IIP1, IIP1,    5,    7,   27,   36,   30,
/* 0xAX */      5,    5, IIP1,    7, IIP1, IIP1,    5,    5, IIP1, IIP1, IIP1,    5,    7,   27,   36,   30,
/* 0xBX */      6,    6, IIP1,    8, IIP1, IIP1,    6,    6, IIP1, IIP1, IIP1,    6,    8,   28,   37,   31,
/* 0xCX */      3,    3, IIP1, IIP1, IIP1, IIP1,    3, IIP1, IIP1, IIP1, IIP1,    3, IIP1, IIP1, IIP1, IIP1,
/* 0xDX */      5,    5, IIP1, IIP1, IIP1, IIP1,    5,    5, IIP1, IIP1, IIP1,    5, IIP1, IIP1, IIP1, IIP1,
/* 0xEX */      5,    5, IIP1, IIP1, IIP1, IIP1,    5,    5, IIP1, IIP1, IIP1,    5, IIP1, IIP1, IIP1, IIP1,
/* 0xFX */      6,    6, IIP1, IIP1, IIP1, IIP1,    6,    6, IIP1, IIP1, IIP1,    6, IIP1, IIP1, IIP1, IIP1
};

static const UINT8 ccounts_page11_na[256] =    /* Cycle Counts Page 11, Native 6309 */
{
/*           0xX0, 0xX1, 0xX2, 0xX3, 0xX4, 0xX5, 0xX6, 0xX7, 0xX8, 0xX9, 0xXA, 0xXB, 0xXC, 0xXD, 0xXE, 0xXF */
/* 0x0X */   IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1,
/* 0x1X */   IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1,
/* 0x2X */   IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1,
/* 0x3X */      6,    6,    6,    6,    6,    6,    6,    7,    3,    3,    3,    3,    4,    5, IIP1,   22,
/* 0x4X */   IIP1, IIP1, IIP1,    2, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1,    2, IIP1,    2,    2, IIP1,    2,
/* 0x5X */   IIP1, IIP1, IIP1,    2, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1,    2, IIP1,    2,    2, IIP1,    2,
/* 0x6X */   IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1,
/* 0x7X */   IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1, IIP1,
/* 0x8X */      3,    3, IIP1,    4, IIP1, IIP1,    3, IIP1, IIP1, IIP1, IIP1,    3,    4,   25,   34,   28,
/* 0x9X */      4,    4, IIP1,    5, IIP1, IIP1,    4,    4, IIP1, IIP1, IIP1,    4,    5,   26,   35,   29,
/* 0xAX */      5,    5, IIP1,    6, IIP1, IIP1,    5,    5, IIP1, IIP1, IIP1,    5,    6,   27,   36,   30,
/* 0xBX */      5,    5, IIP1,    6, IIP1, IIP1,    5,    5, IIP1, IIP1, IIP1,    5,    6,   27,   36,   30,
/* 0xCX */      3,    3, IIP1, IIP1, IIP1, IIP1,    3, IIP1, IIP1, IIP1, IIP1,    3, IIP1, IIP1, IIP1, IIP1,
/* 0xDX */      4,    4, IIP1, IIP1, IIP1, IIP1,    4,    4, IIP1, IIP1, IIP1,    4, IIP1, IIP1, IIP1, IIP1,
/* 0xEX */      5,    5, IIP1, IIP1, IIP1, IIP1,    5,    5, IIP1, IIP1, IIP1,    5, IIP1, IIP1, IIP1, IIP1,
/* 0xFX */      5,    5, IIP1, IIP1, IIP1, IIP1,    5,    5, IIP1, IIP1, IIP1,    5, IIP1, IIP1, IIP1, IIP1
};

#ifndef BIG_SWITCH

static void (*const hd6309_main[0x100])(void) = {
/*          0xX0,   0xX1,     0xX2,    0xX3,    0xX4,    0xX5,    0xX6,    0xX7,
            0xX8,   0xX9,     0xXA,    0xXB,    0xXC,    0xXD,    0xXE,    0xXF   */

/* 0x0X */  neg_di,  oim_di,  aim_di,  com_di,  lsr_di,  eim_di,  ror_di,  asr_di,
            asl_di,  rol_di,  dec_di,  tim_di,  inc_di,  tst_di,  jmp_di,  clr_di,

/* 0x1X */  pref10,  pref11,  nop,     sync,    sexw,    IIError, lbra,    lbsr,
            IIError, daa,     orcc,    IIError, andcc,   sex,     exg,     tfr,

/* 0x2X */  bra,     brn,     bhi,     bls,     bcc,     bcs,     bne,     beq,
            bvc,     bvs,     bpl,     bmi,     bge,     blt,     bgt,     ble,

/* 0x3X */  leax,    leay,    leas,    leau,    pshs,    puls,    pshu,    pulu,
            IIError, rts,     abx,     rti,     cwai,    mul,     IIError, swi,

/* 0x4X */  nega,    IIError, IIError, coma,    lsra,    IIError, rora,    asra,
            asla,    rola,    deca,    IIError, inca,    tsta,    IIError, clra,

/* 0x5X */  negb,    IIError, IIError, comb,    lsrb,    IIError, rorb,    asrb,
            aslb,    rolb,    decb,    IIError, incb,    tstb,    IIError, clrb,

/* 0x6X */  neg_ix,  oim_ix,  aim_ix,  com_ix,  lsr_ix,  eim_ix,  ror_ix,  asr_ix,
            asl_ix,  rol_ix,  dec_ix,  tim_ix,  inc_ix,  tst_ix,  jmp_ix,  clr_ix,

/* 0x7X */  neg_ex,  oim_ex,  aim_ex,  com_ex,  lsr_ex,  eim_ex,  ror_ex,  asr_ex,
            asl_ex,  rol_ex,  dec_ex,  tim_ex,  inc_ex,  tst_ex,  jmp_ex,  clr_ex,

/* 0x8X */  suba_im, cmpa_im, sbca_im, subd_im, anda_im, bita_im, lda_im,  IIError,
            eora_im, adca_im, ora_im,  adda_im, cmpx_im, bsr,     ldx_im,  IIError,

/* 0x9X */  suba_di, cmpa_di, sbca_di, subd_di, anda_di, bita_di, lda_di,  sta_di,
            eora_di, adca_di, ora_di,  adda_di, cmpx_di, jsr_di,  ldx_di,  stx_di,

/* 0xAX */  suba_ix, cmpa_ix, sbca_ix, subd_ix, anda_ix, bita_ix, lda_ix,  sta_ix,
            eora_ix, adca_ix, ora_ix,  adda_ix, cmpx_ix, jsr_ix,  ldx_ix,  stx_ix,

/* 0xBX */  suba_ex, cmpa_ex, sbca_ex, subd_ex, anda_ex, bita_ex, lda_ex,  sta_ex,
            eora_ex, adca_ex, ora_ex,  adda_ex, cmpx_ex, jsr_ex,  ldx_ex,  stx_ex,

/* 0xCX */  subb_im, cmpb_im, sbcb_im, addd_im, andb_im, bitb_im, ldb_im,  IIError,
            eorb_im, adcb_im, orb_im,  addb_im, ldd_im,  ldq_im,  ldu_im,  IIError,

/* 0xDX */  subb_di, cmpb_di, sbcb_di, addd_di, andb_di, bitb_di, ldb_di,  stb_di,
            eorb_di, adcb_di, orb_di,  addb_di, ldd_di,  std_di,  ldu_di,  stu_di,

/* 0xEX */  subb_ix, cmpb_ix, sbcb_ix, addd_ix, andb_ix, bitb_ix, ldb_ix,  stb_ix,
            eorb_ix, adcb_ix, orb_ix,  addb_ix, ldd_ix,  std_ix,  ldu_ix,  stu_ix,

/* 0xFX */  subb_ex, cmpb_ex, sbcb_ex, addd_ex, andb_ex, bitb_ex, ldb_ex,  stb_ex,
            eorb_ex, adcb_ex, orb_ex,  addb_ex, ldd_ex,  std_ex,  ldu_ex,  stu_ex
};

static void (*const hd6309_page01[0x100])(void) = {
/*          0xX0,   0xX1,     0xX2,    0xX3,    0xX4,    0xX5,    0xX6,    0xX7,
            0xX8,   0xX9,     0xXA,    0xXB,    0xXC,    0xXD,    0xXE,    0xXF   */

/* 0x0X */  IIError, IIError, IIError, IIError, IIError, IIError, IIError, IIError,
			IIError, IIError, IIError, IIError, IIError, IIError, IIError, IIError,

/* 0x1X */  IIError, IIError, IIError, IIError, IIError, IIError, IIError, IIError,
			IIError, IIError, IIError, IIError, IIError, IIError, IIError, IIError,

/* 0x2X */  IIError, lbrn,    lbhi,    lbls,    lbcc,    lbcs,    lbne,    lbeq,
			lbvc,    lbvs,    lbpl,    lbmi,    lbge,    lblt,    lbgt,    lble,

/* 0x3X */  addr_r,  adcr,    subr,    sbcr,    andr,    orr,     eorr,    cmpr,
			pshsw,   pulsw,   pshuw,   puluw,   IIError, IIError, IIError, swi2,

/* 0x4X */  negd,    IIError, IIError, comd,    lsrd,    IIError, rord,    asrd,
			asld,    rold,    decd,    IIError, incd,    tstd,    IIError, clrd,

/* 0x5X */  IIError, IIError, IIError, comw,    lsrw,    IIError, rorw,    IIError,
			IIError, rolw,    decw,    IIError, incw,    tstw,    IIError, clrw,

/* 0x6X */  IIError, IIError, IIError, IIError, IIError, IIError, IIError, IIError,
			IIError, IIError, IIError, IIError, IIError, IIError, IIError, IIError,

/* 0x7X */  IIError, IIError, IIError, IIError, IIError, IIError, IIError, IIError,
			IIError, IIError, IIError, IIError, IIError, IIError, IIError, IIError,

/* 0x8X */  subw_im, cmpw_im, sbcd_im, cmpd_im, andd_im, bitd_im, ldw_im,  IIError,
			eord_im, adcd_im, ord_im,  addw_im, cmpy_im, IIError, ldy_im,  IIError,

/* 0x9X */  subw_di, cmpw_di, sbcd_di, cmpd_di, andd_di, bitd_di, ldw_di,  stw_di,
			eord_di, adcd_di, ord_di,  addw_di, cmpy_di, IIError, ldy_di,  sty_di,

/* 0xAX */  subw_ix, cmpw_ix, sbcd_ix, cmpd_ix, andd_ix, bitd_ix, ldw_ix,  stw_ix,
			eord_ix, adcd_ix, ord_ix,  addw_ix, cmpy_ix, IIError, ldy_ix,  sty_ix,

/* 0xBX */  subw_ex, cmpw_ex, sbcd_ex, cmpd_ex, andd_ex, bitd_ex, ldw_ex,  stw_ex,
			eord_ex, adcd_ex, ord_ex,  addw_ex, cmpy_ex, IIError, ldy_ex,  sty_ex,

/* 0xCX */  IIError, IIError, IIError, IIError, IIError, IIError, IIError, IIError,
			IIError, IIError, IIError, IIError, IIError, IIError, lds_im,  IIError,

/* 0xDX */  IIError, IIError, IIError, IIError, IIError, IIError, IIError, IIError,
			IIError, IIError, IIError, IIError, ldq_di,  stq_di,  lds_di,  sts_di,

/* 0xEX */  IIError, IIError, IIError, IIError, IIError, IIError, IIError, IIError,
			IIError, IIError, IIError, IIError, ldq_ix,  stq_ix,  lds_ix,  sts_ix,

/* 0xFX */  IIError, IIError, IIError, IIError, IIError, IIError, IIError, IIError,
			IIError, IIError, IIError, IIError, ldq_ex,  stq_ex,  lds_ex,  sts_ex
};
static void (*const hd6309_page11[0x100])(void) = {
/*          0xX0,   0xX1,     0xX2,    0xX3,    0xX4,    0xX5,    0xX6,    0xX7,
            0xX8,   0xX9,     0xXA,    0xXB,    0xXC,    0xXD,    0xXE,    0xXF   */

/* 0x0X */  IIError, IIError, IIError, IIError, IIError, IIError, IIError, IIError,
			IIError, IIError, IIError, IIError, IIError, IIError, IIError, IIError,

/* 0x1X */  IIError, IIError, IIError, IIError, IIError, IIError, IIError, IIError,
			IIError, IIError, IIError, IIError, IIError, IIError, IIError, IIError,

/* 0x2X */  IIError, IIError, IIError, IIError, IIError, IIError, IIError, IIError,
			IIError, IIError, IIError, IIError, IIError, IIError, IIError, IIError,

/* 0x3X */  band,    biand,   bor,     bior,    beor,    bieor,   ldbt,    stbt,
			tfmpp,   tfmmm,   tfmpc,   tfmcp,   bitmd_im,ldmd_im, IIError, swi3,

/* 0x4X */  IIError, IIError, IIError, come,    IIError, IIError, IIError, IIError,
			IIError, IIError, dece,    IIError, ince,    tste,    IIError, clre,

/* 0x5X */  IIError, IIError, IIError, comf,    IIError, IIError, IIError, IIError,
			IIError, IIError, decf,    IIError, incf,    tstf,    IIError, clrf,

/* 0x6X */  IIError, IIError, IIError, IIError, IIError, IIError, IIError, IIError,
			IIError, IIError, IIError, IIError, IIError, IIError, IIError, IIError,

/* 0x7X */  IIError, IIError, IIError, IIError, IIError, IIError, IIError, IIError,
			IIError, IIError, IIError, IIError, IIError, IIError, IIError, IIError,

/* 0x8X */  sube_im, cmpe_im, IIError, cmpu_im, IIError, IIError, lde_im,  IIError,
			IIError, IIError, IIError, adde_im, cmps_im, divd_im, divq_im, muld_im,

/* 0x9X */  sube_di, cmpe_di, IIError, cmpu_di, IIError, IIError, lde_di,  ste_di,
			IIError, IIError, IIError, adde_di, cmps_di, divd_di, divq_di, muld_di,

/* 0xAX */  sube_ix, cmpe_ix, IIError, cmpu_ix, IIError, IIError, lde_ix,  ste_ix,
			IIError, IIError, IIError, adde_ix, cmps_ix, divd_ix, divq_ix, muld_ix,

/* 0xBX */  sube_ex, cmpe_ex, IIError, cmpu_ex, IIError, IIError, lde_ex,  ste_ex,
			IIError, IIError, IIError, adde_ex, cmps_ex, divd_ex, divq_ex, muld_ex,

/* 0xCX */  subf_im, cmpf_im, IIError, IIError, IIError, IIError, ldf_im,  IIError,
			IIError, IIError, IIError, addf_im, IIError, IIError, IIError, IIError,

/* 0xDX */  subf_di, cmpf_di, IIError, IIError, IIError, IIError, ldf_di,  stf_di,
			IIError, IIError, IIError, addf_di, IIError, IIError, IIError, IIError,

/* 0xEX */  subf_ix, cmpf_ix, IIError, IIError, IIError, IIError, ldf_ix,  stf_ix,
			IIError, IIError, IIError, addf_ix, IIError, IIError, IIError, IIError,

/* 0xFX */  subf_ex, cmpf_ex, IIError, IIError, IIError, IIError, ldf_ex,  stf_ex,
			IIError, IIError, IIError, addf_ex, IIError, IIError, IIError, IIError

};

#endif /* BIG_SWITCH */


#ifndef __cplusplus
}
#endif

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

#define LBRANCH(f) {					\
	PAIR t; 							\
	IMMWORD(t); 						\
	if( f ) 							\
	{									\
		if( !(MD & MD_EM) )				\
			hd6309_ICount -= 1;			\
		PC += t.w.l;					\
		CHANGE_PC;						\
	}									\
}

HD6309_INLINE UINT32 RM16( UINT32 mAddr );
HD6309_INLINE UINT32 RM16( UINT32 mAddr )
{
	UINT32 result = RM(mAddr) << 8;
	return result | RM((mAddr+1)&0xffff);
}

HD6309_INLINE UINT32 RM32( UINT32 mAddr );
HD6309_INLINE UINT32 RM32( UINT32 mAddr )
{
	UINT32 result = RM(mAddr) << 24;
	result += RM(mAddr+1) << 16;
	result += RM(mAddr+2) << 8;
	result += RM(mAddr+3);
	return result;
}

HD6309_INLINE void WM16( UINT32 mAddr, PAIR *p );
HD6309_INLINE void WM16( UINT32 mAddr, PAIR *p )
{
	WM( mAddr, p->b.h );
	WM( (mAddr+1)&0xffff, p->b.l );
}

HD6309_INLINE void WM32( UINT32 mAddr, PAIR *p );
HD6309_INLINE void WM32( UINT32 mAddr, PAIR *p )
{
	WM( mAddr, p->b.h3 );
	WM( (mAddr+1)&0xffff, p->b.h2 );
	WM( (mAddr+2)&0xffff, p->b.h );
	WM( (mAddr+3)&0xffff, p->b.l );
}

static void UpdateState(void)
{
	if ( hd6309.md & MD_EM )
	{
		cycle_counts_page0  = ccounts_page0_na;
		cycle_counts_page01 = ccounts_page01_na;
		cycle_counts_page11 = ccounts_page11_na;
		index_cycle         = index_cycle_na;
	}
	else
	{
		cycle_counts_page0  = ccounts_page0_em;
		cycle_counts_page01 = ccounts_page01_em;
		cycle_counts_page11 = ccounts_page11_em;
		index_cycle         = index_cycle_em;
	}
}

extern "C" 
{
static void CHECK_IRQ_LINES( void )
{
	if( hd6309.irq_state[HD6309_IRQ_LINE] != HD6309_CLEAR_LINE ||
		hd6309.irq_state[HD6309_FIRQ_LINE] != HD6309_CLEAR_LINE )
		hd6309.int_state &= ~HD6309_SYNC; /* clear SYNC flag */
	if( hd6309.irq_state[HD6309_FIRQ_LINE]!=HD6309_CLEAR_LINE && !(CC & CC_IF))
	{
		/* fast IRQ */
		/* HJB 990225: state already saved by CWAI? */
		if( hd6309.int_state & HD6309_CWAI )
		{
			hd6309.int_state &= ~HD6309_CWAI;
			hd6309.extra_cycles += 7;		 /* subtract +7 cycles */
		}
		else
		{
			if ( MD & MD_FM )
			{
				CC |= CC_E; 				/* save entire state */
				PUSHWORD(pPC);
				PUSHWORD(pU);
				PUSHWORD(pY);
				PUSHWORD(pX);
				PUSHBYTE(DP);
				if ( MD & MD_EM )
				{
					PUSHBYTE(F);
					PUSHBYTE(E);
					hd6309.extra_cycles += 2; /* subtract +2 cycles */
				}
				PUSHBYTE(B);
				PUSHBYTE(A);
				PUSHBYTE(CC);
				hd6309.extra_cycles += 19;	 /* subtract +19 cycles */
			}
			else
			{
				CC &= ~CC_E;				/* save 'short' state */
				PUSHWORD(pPC);
				PUSHBYTE(CC);
				hd6309.extra_cycles += 10;	/* subtract +10 cycles */
			}
		}
		CC |= CC_IF | CC_II;			/* inhibit FIRQ and IRQ */
		PCD=RM16(0xfff6);
		CHANGE_PC;
//		(void)(*hd6309.irq_callback)(HD6309_FIRQ_LINE);
	}
	else
	if( hd6309.irq_state[HD6309_IRQ_LINE]!=HD6309_CLEAR_LINE && !(CC & CC_II) )
	{
		/* standard IRQ */
		/* HJB 990225: state already saved by CWAI? */
		if( hd6309.int_state & HD6309_CWAI )
		{
			hd6309.int_state &= ~HD6309_CWAI;  /* clear CWAI flag */
			hd6309.extra_cycles += 7;		 /* subtract +7 cycles */
		}
		else
		{
			CC |= CC_E; 				/* save entire state */
			PUSHWORD(pPC);
			PUSHWORD(pU);
			PUSHWORD(pY);
			PUSHWORD(pX);
			PUSHBYTE(DP);
			if ( MD & MD_EM )
			{
				PUSHBYTE(F);
				PUSHBYTE(E);
				hd6309.extra_cycles += 2; /* subtract +2 cycles */
			}
			PUSHBYTE(B);
			PUSHBYTE(A);
			PUSHBYTE(CC);
			hd6309.extra_cycles += 19;	 /* subtract +19 cycles */
		}
		CC |= CC_II;					/* inhibit IRQ */
		PCD=RM16(0xfff8);
		CHANGE_PC;
//		(void)(*hd6309.irq_callback)(HD6309_IRQ_LINE);
	}
}

/****************************************************************************
 * Get all registers in given buffer
 ****************************************************************************/
void hd6309_get_context(void *dst)
{
	if( dst )
		*(hd6309_Regs*)dst = hd6309;
}

/****************************************************************************
 * Set all registers to given values
 ****************************************************************************/
void hd6309_set_context(void *src)
{
	if( src )
		hd6309 = *(hd6309_Regs*)src;
	CHANGE_PC;

	CHECK_IRQ_LINES();
	UpdateState();
}
}

//static STATE_POSTLOAD( hd6309_postload )
//{
//	UpdateState();
//}

void hd6309_init()
{
//	hd6309.irq_callback = irqcallback;

//	state_save_register_item("hd6309", index, PC);
//	state_save_register_item("hd6309", index, U);
//	state_save_register_item("hd6309", index, S);
//	state_save_register_item("hd6309", index, X);
//	state_save_register_item("hd6309", index, Y);
//	state_save_register_item("hd6309", index, V);
//	state_save_register_item("hd6309", index, DP);
//	state_save_register_item("hd6309", index, CC);
//	state_save_register_item("hd6309", index, MD);
//	state_save_register_postload(Machine, hd6309_postload, NULL);
//	state_save_register_item("hd6309", index, hd6309.int_state);
//	state_save_register_item("hd6309", index, hd6309.nmi_state);
//	state_save_register_item("hd6309", index, hd6309.irq_state[0]);
//	state_save_register_item("hd6309", index, hd6309.irq_state[1]);
}

/****************************************************************************/
/* Reset registers to their initial values                                  */
/****************************************************************************/
void hd6309_reset(void)
{
	hd6309.int_state = 0;
	hd6309.nmi_state = HD6309_CLEAR_LINE;
	hd6309.irq_state[0] = HD6309_CLEAR_LINE;
	hd6309.irq_state[0] = HD6309_CLEAR_LINE;

	DPD = 0;			/* Reset direct page register */

	MD = 0; 			/* Mode register gets reset */
	CC |= CC_II;		/* IRQ disabled */
	CC |= CC_IF;		/* FIRQ disabled */

	PCD = RM16(0xfffe);
	CHANGE_PC;
	UpdateState();
}

int hd6309_get_pc()
{
	return PC;
}

/*
static void hd6309_exit(void)
{

}*/

/* Generate interrupts */
/****************************************************************************
 * Set IRQ line state
 ****************************************************************************/
void hd6309_set_irq_line(int irqline, int state)
{
	if (irqline == HD6309_INPUT_LINE_NMI)
	{
		if (hd6309.nmi_state == state) return;
		hd6309.nmi_state = state;
//		LOG(("HD6309#%d set_irq_line (NMI) %d (PC=%4.4X)\n", cpu_getactivecpu(), state, pPC.d));
		if( state == HD6309_CLEAR_LINE ) return;

		/* if the stack was not yet initialized */
		if( !(hd6309.int_state & HD6309_LDS) ) return;

		hd6309.int_state &= ~HD6309_SYNC;
		/* HJB 990225: state already saved by CWAI? */
		if( hd6309.int_state & HD6309_CWAI )
		{
			hd6309.int_state &= ~HD6309_CWAI;
			hd6309.extra_cycles += 7;	/* subtract +7 cycles next time */
		}
		else
		{
			CC |= CC_E; 				/* save entire state */
			PUSHWORD(pPC);
			PUSHWORD(pU);
			PUSHWORD(pY);
			PUSHWORD(pX);
			PUSHBYTE(DP);
			if ( MD & MD_EM )
			{
				PUSHBYTE(F);
				PUSHBYTE(E);
				hd6309.extra_cycles += 2; /* subtract +2 cycles */
			}

			PUSHBYTE(B);
			PUSHBYTE(A);
			PUSHBYTE(CC);
			hd6309.extra_cycles += 19;	/* subtract +19 cycles next time */
		}
		CC |= CC_IF | CC_II;			/* inhibit FIRQ and IRQ */
		PCD = RM16(0xfffc);
		CHANGE_PC;
	}
	else if (irqline < 2)
	{
//		LOG(("HD6309#%d set_irq_line %d, %d (PC=%4.4X)\n", cpu_getactivecpu(), irqline, state, pPC.d));
		hd6309.irq_state[irqline] = state;
		if (state == HD6309_CLEAR_LINE) return;
		CHECK_IRQ_LINES();
	}
}

/* includes the actual opcode implementations */
#ifdef __cplusplus
extern "C" {
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

HD6309_INLINE void illegal( void )
{
//	LOG(("HD6309: illegal opcode at %04x\nVectoring to [$fff0]\n",PC));

	CC |= CC_E;
	PUSHWORD(pPC);
	PUSHWORD(pU);
	PUSHWORD(pY);
	PUSHWORD(pX);
	PUSHBYTE(DP);

	if ( MD & MD_EM )
	{
		PUSHBYTE(F);
		PUSHBYTE(E);
		hd6309_ICount -= 2;
	}

	PUSHBYTE(B);
	PUSHBYTE(A);
	PUSHBYTE(CC);

	PCD = RM16(0xfff0);
	CHANGE_PC;
}

static void IIError(void)
{
	SEII;			// Set illegal Instruction Flag
	illegal();		// Vector to Trap handler
}

static void DZError(void)
{
	SEDZ;			// Set Division by Zero Flag
	illegal();		// Vector to Trap handler
}

/* $00 NEG direct ?**** */
HD6309_INLINE void neg_di( void )
{
	UINT16 r,t;
	DIRBYTE(t);
	r = -t;
	CLR_NZVC;
	SET_FLAGS8(0,t,r);
	WM(EAD,r);
}

/* $01 OIM direct ?**** */
HD6309_INLINE void oim_di( void )
{
	UINT8	r,t,im;
	IMMBYTE(im);
	DIRBYTE(t);
	r = im | t;
	CLR_NZV;
	SET_NZ8(r);
	WM(EAD,r);
}

/* $02 AIM direct */
HD6309_INLINE void aim_di( void )
{
	UINT8	r,t,im;
	IMMBYTE(im);
	DIRBYTE(t);
	r = im & t;
	CLR_NZV;
	SET_NZ8(r);
	WM(EAD,r);
}

/* $03 COM direct -**01 */
HD6309_INLINE void com_di( void )
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
HD6309_INLINE void lsr_di( void )
{
	UINT8 t;
	DIRBYTE(t);
	CLR_NZC;
	CC |= (t & CC_C);
	t >>= 1;
	SET_Z8(t);
	WM(EAD,t);
}

/* $05 EIM direct */
HD6309_INLINE void eim_di( void )
{
	UINT8	r,t,im;
	IMMBYTE(im);
	DIRBYTE(t);
	r = im ^ t;
	CLR_NZV;
	SET_NZ8(r);
	WM(EAD,r);
}

/* $06 ROR direct -**-* */
HD6309_INLINE void ror_di( void )
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
HD6309_INLINE void asr_di( void )
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
HD6309_INLINE void asl_di( void )
{
	UINT16 t,r;
	DIRBYTE(t);
	r = t << 1;
	CLR_NZVC;
	SET_FLAGS8(t,t,r);
	WM(EAD,r);
}

/* $09 ROL direct -**** */
HD6309_INLINE void rol_di( void )
{
	UINT16 t,r;
	DIRBYTE(t);
	r = (CC & CC_C) | (t << 1);
	CLR_NZVC;
	SET_FLAGS8(t,t,r);
	WM(EAD,r);
}

/* $0A DEC direct -***- */
HD6309_INLINE void dec_di( void )
{
	UINT8 t;
	DIRBYTE(t);
	--t;
	CLR_NZV;
	SET_FLAGS8D(t);
	WM(EAD,t);
}

/* $0B TIM direct */
HD6309_INLINE void tim_di( void )
{
	UINT8	r,t,im;
	IMMBYTE(im);
	DIRBYTE(t);
	r = im & t;
	CLR_NZV;
	SET_NZ8(r);
}

/* $OC INC direct -***- */
HD6309_INLINE void inc_di( void )
{
	UINT8 t;
	DIRBYTE(t);
	++t;
	CLR_NZV;
	SET_FLAGS8I(t);
	WM(EAD,t);
}

/* $OD TST direct -**0- */
HD6309_INLINE void tst_di( void )
{
	UINT8 t;
	DIRBYTE(t);
	CLR_NZV;
	SET_NZ8(t);
}

/* $0E JMP direct ----- */
HD6309_INLINE void jmp_di( void )
{
	DIRECT;
	PCD = EAD;
	CHANGE_PC;
}

/* $0F CLR direct -0100 */
HD6309_INLINE void clr_di( void )
{
	UINT32 dummy;
	DIRECT;
	dummy = RM(EAD);
	WM(EAD,0);
	CLR_NZVC;
	SEZ;
}

/* $10 FLAG */

/* $11 FLAG */

/* $12 NOP inherent ----- */
HD6309_INLINE void nop( void )
{
	;
}

/* $13 SYNC inherent ----- */
HD6309_INLINE void sync( void )
{
	/* SYNC stops processing instructions until an interrupt request happens. */
	/* This doesn't require the corresponding interrupt to be enabled: if it */
	/* is disabled, execution continues with the next instruction. */
	hd6309.int_state |= HD6309_SYNC;	 /* HJB 990227 */
	CHECK_IRQ_LINES();
	/* if HD6309_SYNC has not been cleared by CHECK_IRQ_LINES(),
     * stop execution until the interrupt lines change. */
	if( hd6309.int_state & HD6309_SYNC )
		if (hd6309_ICount > 0) hd6309_ICount = 0;
}

/* $14 sexw inherent */
HD6309_INLINE void sexw( void )
{
	PAIR q;
	q.d = SIGNED_16(W);
	D = q.w.h;
	W = q.w.l;
	CLR_NZ;
	SET_N16(D);
	SET_Z(q.d);
}

/* $15 ILLEGAL */

/* $16 LBRA relative ----- */
HD6309_INLINE void lbra( void )
{
	IMMWORD(ea);
	PC += EA;
	CHANGE_PC;

	if ( EA == 0xfffd )  /* EHC 980508 speed up busy loop */
		if ( hd6309_ICount > 0)
			hd6309_ICount = 0;
}

/* $17 LBSR relative ----- */
HD6309_INLINE void lbsr( void )
{
	IMMWORD(ea);
	PUSHWORD(pPC);
	PC += EA;
	CHANGE_PC;
}

/* $18 ILLEGAL */

/* $19 DAA inherent (A) -**0* */
HD6309_INLINE void daa( void )
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
HD6309_INLINE void orcc( void )
{
	UINT8 t;
	IMMBYTE(t);
	CC |= t;
	CHECK_IRQ_LINES();	/* HJB 990116 */
}

/* $1B ILLEGAL */

/* $1C ANDCC immediate ##### */
HD6309_INLINE void andcc( void )
{
	UINT8 t;
	IMMBYTE(t);
	CC &= t;
	CHECK_IRQ_LINES();	/* HJB 990116 */
}

/* $1D SEX inherent -**0- */
HD6309_INLINE void sex( void )
{
	UINT16 t;
	t = SIGNED(B);
	D = t;
	CLR_NZ;
	SET_NZ16(t);
}

HD6309_INLINE void exg( void )
{
	UINT16 t1,t2;
	UINT8 tb;
	int 	promote = FALSE;

	IMMBYTE(tb);
	if( (tb^(tb>>4)) & 0x08 )	/* HJB 990225: mixed 8/16 bit case? */
	{
		promote = TRUE;
	}

	switch(tb>>4) {
		case  0: t1 = D;  break;
		case  1: t1 = X;  break;
		case  2: t1 = Y;  break;
		case  3: t1 = U;  break;
		case  4: t1 = S;  break;
		case  5: t1 = PC; break;
		case  6: t1 = W;  break;
		case  7: t1 = V;  break;
		case  8: t1 = (promote ? A + ((A) << 8) : A);  break;
		case  9: t1 = (promote ? B + ((B) << 8) : B);  break;
		case 10: t1 = (promote ? CC + ((CC) << 8) : CC); break;
		case 11: t1 = (promote ? DP + ((DP) << 8) : DP); break;
		case 12: t1 = 0;  break;
		case 13: t1 = 0;  break;
		case 14: t1 = (promote ? E + ((E) << 8) : E); break;
		default: t1 = (promote ? F + ((F) << 8) : F); break;
	}
	switch(tb&15) {
		case  0: t2 = D;  break;
		case  1: t2 = X;  break;
		case  2: t2 = Y;  break;
		case  3: t2 = U;  break;
		case  4: t2 = S;  break;
		case  5: t2 = PC; break;
		case  6: t2 = W;  break;
		case  7: t2 = V;  break;
		case  8: t2 = (promote ? A + ((A) << 8) : A);  break;
		case  9: t2 = (promote ? B + ((B) << 8) : B);  break;
		case 10: t2 = (promote ? CC + ((CC) << 8) : CC); break;
		case 11: t2 = (promote ? DP + ((DP) << 8) : DP); break;
		case 12: t2 = 0;  break;
		case 13: t2 = 0;  break;
		case 14: t2 = (promote ? E + ((E) << 8) : E); break;
		default: t2 = (promote ? F + ((F) << 8) : F); break;
	}

	switch(tb>>4) {
		case  0: D = t2;  break;
		case  1: X = t2;  break;
		case  2: Y = t2;  break;
		case  3: U = t2;  break;
		case  4: S = t2;  break;
		case  5: PC = t2; CHANGE_PC; break;
		case  6: W = t2;  break;
		case  7: V = t2;  break;
		case  8: A = (promote ? t2 >> 8 : t2); break;
		case  9: B = (promote ? t2 & 0xff : t2); break;
		case 10: CC = (promote ? t2 & 0xff : t2); break;
		case 11: DP = (promote ? t2 >> 8 : t2); break;
		case 12: /* 0 = t2 */ break;
		case 13: /* 0 = t2 */ break;
		case 14: E = (promote ? t2 >> 8 : t2); break;
		case 15: F = (promote ? t2 & 0xff : t2); break;
	}
	switch(tb&15) {
		case  0: D = t1;  break;
		case  1: X = t1;  break;
		case  2: Y = t1;  break;
		case  3: U = t1;  break;
		case  4: S = t1;  break;
		case  5: PC = t1; CHANGE_PC; break;
		case  6: W = t1;  break;
		case  7: V = t1;  break;
		case  8: A = (promote ? t1 >> 8 : t1); break;
		case  9: B = (promote ? t1 & 0xff : t1); break;
		case 10: CC = (promote ? t1 & 0xff : t1); break;
		case 11: DP = (promote ? t1 >> 8 : t1); break;
		case 12: /* 0 = t1 */ break;
		case 13: /* 0 = t1 */ break;
		case 14: E = (promote ? t1 >> 8 : t1); break;
		case 15: F = (promote ? t1 & 0xff : t1); break;
	}
}

/* $1F TFR inherent ----- */
HD6309_INLINE void tfr( void )
{
	UINT8 tb;
	UINT16 t;
	int 	promote = FALSE;

	IMMBYTE(tb);
	if( (tb^(tb>>4)) & 0x08 )
	{
		promote = TRUE;
	}

	switch(tb>>4) {
		case  0: t = D;  break;
		case  1: t = X;  break;
		case  2: t = Y;  break;
		case  3: t = U;  break;
		case  4: t = S;  break;
		case  5: t = PC; break;
		case  6: t = W;  break;
		case  7: t = V;  break;
		case  8: t = (promote ? A + ((A) << 8) : A);  break;
		case  9: t = (promote ? B + ((B) << 8) : B);  break;
		case 10: t = (promote ? CC + ((CC) << 8) : CC); break;
		case 11: t = (promote ? DP + ((DP) << 8) : DP); break;
		case 12: t = 0;  break;
		case 13: t = 0;  break;
		case 14: t = (promote ? E + ((E) << 8) : E); break;
		default: t = (promote ? F + ((F) << 8) : F); break;
	}

	switch(tb&15) {
		case  0: D = t;  break;
		case  1: X = t;  break;
		case  2: Y = t;  break;
		case  3: U = t;  break;
		case  4: S = t;  break;
		case  5: PC = t; CHANGE_PC; break;
		case  6: W = t;  break;
		case  7: V = t;  break;
		case  8: A = (promote ? t >> 8 : t); break;
		case  9: B = (promote ? t & 0xff : t); break;
		case 10: CC = (promote ? t & 0xff : t); break;
		case 11: DP = (promote ? t >> 8 : t); break;
		case 12: /* 0 = t */ break;
		case 13: /* 0 = t */ break;
		case 14: E = (promote ? t >> 8 : t); break;
		case 15: F = (promote ? t & 0xff : t); break;
	}
}

/* $20 BRA relative ----- */
HD6309_INLINE void bra( void )
{
	UINT8 t;
	IMMBYTE(t);
	PC += SIGNED(t);
	CHANGE_PC;
	/* JB 970823 - speed up busy loops */
	if( t == 0xfe )
		if( hd6309_ICount > 0 ) hd6309_ICount = 0;
}

/* $21 BRN relative ----- */
HD6309_INLINE void brn( void )
{
	UINT8 t;
	IMMBYTE(t);
}

/* $1021 LBRN relative ----- */
HD6309_INLINE void lbrn( void )
{
	IMMWORD(ea);
}

/* $22 BHI relative ----- */
HD6309_INLINE void bhi( void )
{
	BRANCH( !(CC & (CC_Z|CC_C)) );
}

/* $1022 LBHI relative ----- */
HD6309_INLINE void lbhi( void )
{
	LBRANCH( !(CC & (CC_Z|CC_C)) );
}

/* $23 BLS relative ----- */
HD6309_INLINE void bls( void )
{
	BRANCH( (CC & (CC_Z|CC_C)) );
}

/* $1023 LBLS relative ----- */
HD6309_INLINE void lbls( void )
{
	LBRANCH( (CC&(CC_Z|CC_C)) );
}

/* $24 BCC relative ----- */
HD6309_INLINE void bcc( void )
{
	BRANCH( !(CC&CC_C) );
}

/* $1024 LBCC relative ----- */
HD6309_INLINE void lbcc( void )
{
	LBRANCH( !(CC&CC_C) );
}

/* $25 BCS relative ----- */
HD6309_INLINE void bcs( void )
{
	BRANCH( (CC&CC_C) );
}

/* $1025 LBCS relative ----- */
HD6309_INLINE void lbcs( void )
{
	LBRANCH( (CC&CC_C) );
}

/* $26 BNE relative ----- */
HD6309_INLINE void bne( void )
{
	BRANCH( !(CC&CC_Z) );
}

/* $1026 LBNE relative ----- */
HD6309_INLINE void lbne( void )
{
	LBRANCH( !(CC&CC_Z) );
}

/* $27 BEQ relative ----- */
HD6309_INLINE void beq( void )
{
	BRANCH( (CC&CC_Z) );
}

/* $1027 LBEQ relative ----- */
HD6309_INLINE void lbeq( void )
{
	LBRANCH( (CC&CC_Z) );
}

/* $28 BVC relative ----- */
HD6309_INLINE void bvc( void )
{
	BRANCH( !(CC&CC_V) );
}

/* $1028 LBVC relative ----- */
HD6309_INLINE void lbvc( void )
{
	LBRANCH( !(CC&CC_V) );
}

/* $29 BVS relative ----- */
HD6309_INLINE void bvs( void )
{
	BRANCH( (CC&CC_V) );
}

/* $1029 LBVS relative ----- */
HD6309_INLINE void lbvs( void )
{
	LBRANCH( (CC&CC_V) );
}

/* $2A BPL relative ----- */
HD6309_INLINE void bpl( void )
{
	BRANCH( !(CC&CC_N) );
}

/* $102A LBPL relative ----- */
HD6309_INLINE void lbpl( void )
{
	LBRANCH( !(CC&CC_N) );
}

/* $2B BMI relative ----- */
HD6309_INLINE void bmi( void )
{
	BRANCH( (CC&CC_N) );
}

/* $102B LBMI relative ----- */
HD6309_INLINE void lbmi( void )
{
	LBRANCH( (CC&CC_N) );
}

/* $2C BGE relative ----- */
HD6309_INLINE void bge( void )
{
	BRANCH( !NXORV );
}

/* $102C LBGE relative ----- */
HD6309_INLINE void lbge( void )
{
	LBRANCH( !NXORV );
}

/* $2D BLT relative ----- */
HD6309_INLINE void blt( void )
{
	BRANCH( NXORV );
}

/* $102D LBLT relative ----- */
HD6309_INLINE void lblt( void )
{
	LBRANCH( NXORV );
}

/* $2E BGT relative ----- */
HD6309_INLINE void bgt( void )
{
	BRANCH( !(NXORV || (CC&CC_Z)) );
}

/* $102E LBGT relative ----- */
HD6309_INLINE void lbgt( void )
{
	LBRANCH( !(NXORV || (CC&CC_Z)) );
}

/* $2F BLE relative ----- */
HD6309_INLINE void ble( void )
{
	BRANCH( (NXORV || (CC&CC_Z)) );
}

/* $102F LBLE relative ----- */
HD6309_INLINE void lble( void )
{
	LBRANCH( (NXORV || (CC&CC_Z)) );
}

#define REGREG_PREAMBLE														\
	IMMBYTE(tb);															\
	if( (tb^(tb>>4)) & 0x08 )												\
		{promote = TRUE;}													\
	switch(tb>>4) {															\
		case  0: src16Reg = &D; large = TRUE;  break;						\
		case  1: src16Reg = &X; large = TRUE;  break;						\
		case  2: src16Reg = &Y; large = TRUE;  break;						\
		case  3: src16Reg = &U; large = TRUE;  break;						\
		case  4: src16Reg = &S; large = TRUE;  break;						\
		case  5: src16Reg = &PC; large = TRUE; break;						\
		case  6: src16Reg = &W; large = TRUE;  break;						\
		case  7: src16Reg = &V; large = TRUE;  break;						\
		case  8: if (promote) src16Reg = &D; else src8Reg = &A; break;		\
		case  9: if (promote) src16Reg = &D; else src8Reg = &B; break;		\
		case 10: if (promote) src16Reg = &z16; else src8Reg = &CC; break;	\
		case 11: if (promote) src16Reg = &z16; else src8Reg = &DP; break;	\
		case 12: if (promote) src16Reg = &z16; else src8Reg = &z8; break;	\
		case 13: if (promote) src16Reg = &z16; else src8Reg = &z8; break;	\
		case 14: if (promote) src16Reg = &W; else src8Reg = &E; break;		\
		default: if (promote) src16Reg = &W; else src8Reg = &F; break;		\
	}																		\
	switch(tb&15) {															\
		case  0: dst16Reg = &D; large = TRUE;  break;						\
		case  1: dst16Reg = &X; large = TRUE;  break;						\
		case  2: dst16Reg = &Y; large = TRUE;  break;						\
		case  3: dst16Reg = &U; large = TRUE;  break;						\
		case  4: dst16Reg = &S; large = TRUE;  break;						\
		case  5: dst16Reg = &PC; large = TRUE; break;						\
		case  6: dst16Reg = &W; large = TRUE;  break;						\
		case  7: dst16Reg = &V; large = TRUE;  break;						\
		case  8: if (promote) dst16Reg = &D; else dst8Reg = &A; break;		\
		case  9: if (promote) dst16Reg = &D; else dst8Reg = &B; break;		\
		case 10: if (promote) dst16Reg = &z16; else dst8Reg = &CC; break;	\
		case 11: if (promote) dst16Reg = &z16; else dst8Reg = &DP; break;	\
		case 12: if (promote) dst16Reg = &z16; else dst8Reg = &z8; break;	\
		case 13: if (promote) dst16Reg = &z16; else dst8Reg = &z8; break;	\
		case 14: if (promote) dst16Reg = &W; else dst8Reg = &E; break;		\
		default: if (promote) dst16Reg = &W; else dst8Reg = &F; break;		\
	}																		\

/* $1030 addr_r r1 + r2 -> r2 */

HD6309_INLINE void addr_r( void )
{
	UINT8	tb, z8 = 0;
	UINT16	z16 = 0, r8;
	UINT32	r16;
	UINT8	*src8Reg = NULL, *dst8Reg = NULL;
	UINT16	*src16Reg = NULL, *dst16Reg = NULL;
	int 	promote = FALSE, large = FALSE;

	REGREG_PREAMBLE;

	if ( large )
	{
		r16 = *src16Reg + *dst16Reg;
		CLR_NZVC;
		*dst16Reg = r16;
		SET_FLAGS16(*src16Reg,*dst16Reg,r16);

		if ( (tb&15) == 5 )
		{
			CHANGE_PC;
		}
	}
	else
	{
		r8 = *src8Reg + *dst8Reg;
		CLR_NZVC;
		/* SET_H(*src8Reg,*src8Reg,r8);*/ /*Experimentation prooved this not to be the case */
		*dst8Reg = r8;
		SET_FLAGS8(*src8Reg,*dst8Reg,r8);
	}
}

HD6309_INLINE void adcr( void )
{
	UINT8	tb, z8 = 0;
	UINT16	z16 = 0, r8;
	UINT32	r16;
	UINT8	*src8Reg = NULL, *dst8Reg = NULL;
	UINT16	*src16Reg = NULL, *dst16Reg = NULL;
	int 	promote = FALSE, large = FALSE;

	REGREG_PREAMBLE;

	if ( large )
	{
		r16 = *src16Reg + *dst16Reg + (CC & CC_C);
		CLR_NZVC;
		*dst16Reg = r16;
		SET_FLAGS16(*src16Reg,*dst16Reg,r16);

		if ( (tb&15) == 5 )
		{
			CHANGE_PC;
		}
	}
	else
	{
		r8 = *src8Reg + *dst8Reg + (CC & CC_C);
		CLR_NZVC;
		/* SET_H(*src8Reg,*src8Reg,r8);*/ /*Experimentation prooved this not to be the case */
		*dst8Reg = r8;
		SET_FLAGS8(*src8Reg,*dst8Reg,r8);
	}
}

/* $1032 SUBR r1 - r2 -> r2 */
HD6309_INLINE void subr( void )
{
	UINT8	tb, z8 = 0;
	UINT16	z16 = 0, r8;
	UINT32	r16;
	UINT8	*src8Reg = NULL, *dst8Reg = NULL;
	UINT16	*src16Reg = NULL, *dst16Reg = NULL;
	int 	promote = FALSE, large = FALSE;

	REGREG_PREAMBLE;

	if ( large )
	{
		r16 = (UINT32)*dst16Reg - (UINT32)*src16Reg;
		CLR_NZVC;
		*dst16Reg = r16;
		SET_FLAGS16((UINT32)*dst16Reg,(UINT32)*src16Reg,r16);

		if ( (tb&15) == 5 )
		{
			CHANGE_PC;
		}
	}
	else
	{
		r8 = *dst8Reg - *src8Reg;
		CLR_NZVC;
		*dst8Reg = r8;
		SET_FLAGS8(*dst8Reg,*src8Reg,r8);
	}
}

/* $1033 SBCR r1 - r2 - C -> r2 */
HD6309_INLINE void sbcr( void )
{
	UINT8	tb, z8 = 0;
	UINT16	z16 = 0, r8;
	UINT32	r16;
	UINT8	*src8Reg = NULL, *dst8Reg = NULL;
	UINT16	*src16Reg = NULL, *dst16Reg = NULL;
	int 	promote = FALSE, large = FALSE;

	REGREG_PREAMBLE;

	if ( large )
	{
		r16 = (UINT32)*dst16Reg - (UINT32)*src16Reg - (CC & CC_C);
		CLR_NZVC;
		*dst16Reg = r16;
		SET_FLAGS16((UINT32)*dst16Reg,(UINT32)*src16Reg,r16);

		if ( (tb&15) == 5 )
		{
			CHANGE_PC;
		}
	}
	else
	{
		r8 = *dst8Reg - *src8Reg - (CC & CC_C);
		CLR_NZVC;
		*dst8Reg = r8;
		SET_FLAGS8(*dst8Reg,*src8Reg,r8);
	}
}

/* $1034 ANDR r1 & r2 -> r2 */
HD6309_INLINE void andr( void )
{
	UINT8	tb, z8 = 0;
	UINT16	z16 = 0, r8;
	UINT32	r16;
	UINT8	*src8Reg = NULL, *dst8Reg = NULL;
	UINT16	*src16Reg = NULL, *dst16Reg = NULL;
	int 	promote = FALSE, large = FALSE;

	REGREG_PREAMBLE;

	if ( large )
	{
		r16 = *src16Reg & *dst16Reg;
		CLR_NZV;
		*dst16Reg = r16;
		SET_NZ16(r16);

		if ( (tb&15) == 5 )
		{
			CHANGE_PC;
		}
	}
	else
	{
		r8 = *src8Reg & *dst8Reg;
		CLR_NZV;
		*dst8Reg = r8;
		SET_NZ8(r8);
	}
}

/* $1035 ORR r1 | r2 -> r2 */
HD6309_INLINE void orr( void )
{
	UINT8	tb, z8 = 0;
	UINT16	z16 = 0, r8;
	UINT32	r16;
	UINT8	*src8Reg = NULL, *dst8Reg = NULL;
	UINT16	*src16Reg = NULL, *dst16Reg = NULL;
	int 	promote = FALSE, large = FALSE;

	REGREG_PREAMBLE;

	if ( large )
	{
		r16 = *src16Reg | *dst16Reg;
		CLR_NZV;
		*dst16Reg = r16;
		SET_NZ16(r16);

		if ( (tb&15) == 5 )
		{
			CHANGE_PC;
		}
	}
	else
	{
		r8 = *src8Reg | *dst8Reg;
		CLR_NZV;
		*dst8Reg = r8;
		SET_NZ8(r8);
	}
}

/* $1036 EORR r1 ^ r2 -> r2 */
HD6309_INLINE void eorr( void )
{
	UINT8	tb, z8 = 0;
	UINT16	z16 = 0, r8;
	UINT32	r16;
	UINT8	*src8Reg = NULL, *dst8Reg = NULL;
	UINT16	*src16Reg = NULL, *dst16Reg = NULL;
	int 	promote = FALSE, large = FALSE;

	REGREG_PREAMBLE;

	if ( large )
	{
		r16 = *src16Reg ^ *dst16Reg;
		CLR_NZV;
		*dst16Reg = r16;
		SET_NZ16(r16);

		if ( (tb&15) == 5 )
		{
			CHANGE_PC;
		}
	}
	else
	{
		r8 = *src8Reg ^ *dst8Reg;
		CLR_NZV;
		*dst8Reg = r8;
		SET_NZ8(r8);
	}
}

/* $1037 CMPR r1 - r2 */
HD6309_INLINE void cmpr( void )
{
	UINT8	tb, z8 = 0;
	UINT16	z16 = 0, r8;
	UINT32	r16;
	UINT8	*src8Reg = NULL, *dst8Reg = NULL;
	UINT16	*src16Reg = NULL, *dst16Reg = NULL;
	int 	promote = FALSE, large = FALSE;

	REGREG_PREAMBLE;

	if ( large )
	{
		r16 = (UINT32)*dst16Reg - (UINT32)*src16Reg;
		CLR_NZVC;
		SET_FLAGS16((UINT32)*dst16Reg,(UINT32)*src16Reg,r16);
	}
	else
	{
		r8 = *dst8Reg - *src8Reg;
		CLR_NZVC;
		SET_FLAGS8(*dst8Reg,*src8Reg,r8);
	}
}

/* $1138 TFM R0+,R1+ */
HD6309_INLINE void tfmpp( void )
{
	UINT8	tb, srcValue = 0;

	IMMBYTE(tb);

	if ( W != 0 )
	{
		switch(tb>>4) {
			case  0: srcValue = RM(D++); break;
			case  1: srcValue = RM(X++); break;
			case  2: srcValue = RM(Y++); break;
			case  3: srcValue = RM(U++); break;
			case  4: srcValue = RM(S++); break;
			default: IIError(); return; break;		/* reg PC thru F */
		}

		switch(tb&15) {
			case  0: WM(D++, srcValue); break;
			case  1: WM(X++, srcValue); break;
			case  2: WM(Y++, srcValue); break;
			case  3: WM(U++, srcValue); break;
			case  4: WM(S++, srcValue); break;
			default: IIError(); return; break;		/* reg PC thru F */
		}

		PCD = PCD - 3;
		CHANGE_PC;
		W--;
	}
	else
		hd6309_ICount -= 6;   /* Needs six aditional cycles to get the 6+3n */
}

/* $1139 TFM R0-,R1- */
HD6309_INLINE void tfmmm( void )
{
	UINT8	tb, srcValue = 0;

	IMMBYTE(tb);

	if ( W != 0 )
	{
		switch(tb>>4) {
			case  0: srcValue = RM(D--); break;
			case  1: srcValue = RM(X--); break;
			case  2: srcValue = RM(Y--); break;
			case  3: srcValue = RM(U--); break;
			case  4: srcValue = RM(S--); break;
			default: IIError(); return; break;		/* reg PC thru F */
		}

		switch(tb&15) {
			case  0: WM(D--, srcValue); break;
			case  1: WM(X--, srcValue); break;
			case  2: WM(Y--, srcValue); break;
			case  3: WM(U--, srcValue); break;
			case  4: WM(S--, srcValue); break;
			default: IIError(); return; break;		/* reg PC thru F */
		}

		PCD = PCD - 3;
		CHANGE_PC;
		W--;
	}
	else
		hd6309_ICount -= 6;   /* Needs six aditional cycles to get the 6+3n */
}

/* $113A TFM R0+,R1 */
HD6309_INLINE void tfmpc( void )
{
	UINT8	tb, srcValue = 0;

	IMMBYTE(tb);

	if ( W != 0 )
	{
		switch(tb>>4) {
			case  0: srcValue = RM(D++); break;
			case  1: srcValue = RM(X++); break;
			case  2: srcValue = RM(Y++); break;
			case  3: srcValue = RM(U++); break;
			case  4: srcValue = RM(S++); break;
			default: IIError(); return; break;		/* reg PC thru F */
		}

		switch(tb&15) {
			case  0: WM(D, srcValue); break;
			case  1: WM(X, srcValue); break;
			case  2: WM(Y, srcValue); break;
			case  3: WM(U, srcValue); break;
			case  4: WM(S, srcValue); break;
			default: IIError(); return; break;		/* reg PC thru F */
		}

		PCD = PCD - 3;
		CHANGE_PC;
		W--;
	}
	else
		hd6309_ICount -= 6;   /* Needs six aditional cycles to get the 6+3n */
}

/* $113B TFM R0,R1+ */
HD6309_INLINE void tfmcp( void )
{
	UINT8	tb, srcValue = 0;

	IMMBYTE(tb);

	if ( W != 0 )
	{
		switch(tb>>4) {
			case  0: srcValue = RM(D); break;
			case  1: srcValue = RM(X); break;
			case  2: srcValue = RM(Y); break;
			case  3: srcValue = RM(U); break;
			case  4: srcValue = RM(S); break;
			default: IIError(); return; break;		/* reg PC thru F */
		}

		switch(tb&15) {
			case  0: WM(D++, srcValue); break;
			case  1: WM(X++, srcValue); break;
			case  2: WM(Y++, srcValue); break;
			case  3: WM(U++, srcValue); break;
			case  4: WM(S++, srcValue); break;
			default: IIError(); return; break;		/* reg PC thru F */
		}

		PCD = PCD - 3;
		CHANGE_PC;
		W--;
	}
	else
		hd6309_ICount -= 6;   /* Needs six aditional cycles to get the 6+3n */
}

/* $30 LEAX indexed --*-- */
HD6309_INLINE void leax( void )
{
	fetch_effective_address();
	X = EA;
	CLR_Z;
	SET_Z(X);
}

/* $31 LEAY indexed --*-- */
HD6309_INLINE void leay( void )
{
	fetch_effective_address();
	Y = EA;
	CLR_Z;
	SET_Z(Y);
}

/* $32 LEAS indexed ----- */
HD6309_INLINE void leas( void )
{
	fetch_effective_address();
	S = EA;
	hd6309.int_state |= HD6309_LDS;
}

/* $33 LEAU indexed ----- */
HD6309_INLINE void leau( void )
{
	fetch_effective_address();
	U = EA;
}

/* $34 PSHS inherent ----- */
HD6309_INLINE void pshs( void )
{
	UINT8 t;
	IMMBYTE(t);
	if( t&0x80 ) { PUSHWORD(pPC); hd6309_ICount -= 2; }
	if( t&0x40 ) { PUSHWORD(pU);  hd6309_ICount -= 2; }
	if( t&0x20 ) { PUSHWORD(pY);  hd6309_ICount -= 2; }
	if( t&0x10 ) { PUSHWORD(pX);  hd6309_ICount -= 2; }
	if( t&0x08 ) { PUSHBYTE(DP);  hd6309_ICount -= 1; }
	if( t&0x04 ) { PUSHBYTE(B);   hd6309_ICount -= 1; }
	if( t&0x02 ) { PUSHBYTE(A);   hd6309_ICount -= 1; }
	if( t&0x01 ) { PUSHBYTE(CC);  hd6309_ICount -= 1; }
}

/* $1038 PSHSW inherent ----- */
HD6309_INLINE void pshsw( void )
{
	PUSHWORD(pW);
}

/* $103a PSHUW inherent ----- */
HD6309_INLINE void pshuw( void )
{
	PSHUWORD(pW);
}

/* $35 PULS inherent ----- */
HD6309_INLINE void puls( void )
{
	UINT8 t;
	IMMBYTE(t);
	if( t&0x01 ) { PULLBYTE(CC); hd6309_ICount -= 1; }
	if( t&0x02 ) { PULLBYTE(A);  hd6309_ICount -= 1; }
	if( t&0x04 ) { PULLBYTE(B);  hd6309_ICount -= 1; }
	if( t&0x08 ) { PULLBYTE(DP); hd6309_ICount -= 1; }
	if( t&0x10 ) { PULLWORD(XD); hd6309_ICount -= 2; }
	if( t&0x20 ) { PULLWORD(YD); hd6309_ICount -= 2; }
	if( t&0x40 ) { PULLWORD(UD); hd6309_ICount -= 2; }
	if( t&0x80 ) { PULLWORD(PCD); CHANGE_PC; hd6309_ICount -= 2; }

	/* HJB 990225: moved check after all PULLs */
	if( t&0x01 ) { CHECK_IRQ_LINES(); }
}

/* $1039 PULSW inherent ----- */
HD6309_INLINE void pulsw( void )
{
	PULLWORD(W);
}

/* $103b PULUW inherent ----- */
HD6309_INLINE void puluw( void )
{
	PULUWORD(W);
}

/* $36 PSHU inherent ----- */
HD6309_INLINE void pshu( void )
{
	UINT8 t;
	IMMBYTE(t);
	if( t&0x80 ) { PSHUWORD(pPC); hd6309_ICount -= 2; }
	if( t&0x40 ) { PSHUWORD(pS);  hd6309_ICount -= 2; }
	if( t&0x20 ) { PSHUWORD(pY);  hd6309_ICount -= 2; }
	if( t&0x10 ) { PSHUWORD(pX);  hd6309_ICount -= 2; }
	if( t&0x08 ) { PSHUBYTE(DP);  hd6309_ICount -= 1; }
	if( t&0x04 ) { PSHUBYTE(B);   hd6309_ICount -= 1; }
	if( t&0x02 ) { PSHUBYTE(A);   hd6309_ICount -= 1; }
	if( t&0x01 ) { PSHUBYTE(CC);  hd6309_ICount -= 1; }
}

/* 37 PULU inherent ----- */
HD6309_INLINE void pulu( void )
{
	UINT8 t;
	IMMBYTE(t);
	if( t&0x01 ) { PULUBYTE(CC); hd6309_ICount -= 1; }
	if( t&0x02 ) { PULUBYTE(A);  hd6309_ICount -= 1; }
	if( t&0x04 ) { PULUBYTE(B);  hd6309_ICount -= 1; }
	if( t&0x08 ) { PULUBYTE(DP); hd6309_ICount -= 1; }
	if( t&0x10 ) { PULUWORD(XD); hd6309_ICount -= 2; }
	if( t&0x20 ) { PULUWORD(YD); hd6309_ICount -= 2; }
	if( t&0x40 ) { PULUWORD(SD); hd6309_ICount -= 2; }
	if( t&0x80 ) { PULUWORD(PCD); CHANGE_PC; hd6309_ICount -= 2; }

	/* HJB 990225: moved check after all PULLs */
	if( t&0x01 ) { CHECK_IRQ_LINES(); }
}

/* $38 ILLEGAL */

/* $39 RTS inherent ----- */
HD6309_INLINE void rts( void )
{
	PULLWORD(PCD);
	CHANGE_PC;
}

/* $3A ABX inherent ----- */
HD6309_INLINE void abx( void )
{
	X += B;
}

/* $3B RTI inherent ##### */
HD6309_INLINE void rti( void )
{
	UINT8 t;
	PULLBYTE(CC);
	t = CC & CC_E;		/* HJB 990225: entire state saved? */
	if(t)
	{
		hd6309_ICount -= 9;
		PULLBYTE(A);
		PULLBYTE(B);
		if ( MD & MD_EM )
		{
			PULLBYTE(E);
			PULLBYTE(F);
			hd6309_ICount -= 2;
		}
		PULLBYTE(DP);
		PULLWORD(XD);
		PULLWORD(YD);
		PULLWORD(UD);
	}
	PULLWORD(PCD);
	CHANGE_PC;
	CHECK_IRQ_LINES();	/* HJB 990116 */
}

/* $3C CWAI inherent ----1 */
HD6309_INLINE void cwai( void )
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
	if ( MD & MD_EM )
	{
		PUSHBYTE(E);
		PUSHBYTE(F);
	}
	PUSHBYTE(B);
	PUSHBYTE(A);
	PUSHBYTE(CC);
	hd6309.int_state |= HD6309_CWAI;	 /* HJB 990228 */
	CHECK_IRQ_LINES();	  /* HJB 990116 */
	if( hd6309.int_state & HD6309_CWAI )
		if( hd6309_ICount > 0 )
			hd6309_ICount = 0;
}

/* $3D MUL inherent --*-@ */
HD6309_INLINE void mul( void )
{
	UINT16 t;
	t = A * B;
	CLR_ZC; SET_Z16(t); if(t&0x80) SEC;
	D = t;
}

/* $3E ILLEGAL */

/* $3F SWI (SWI2 SWI3) absolute indirect ----- */
HD6309_INLINE void swi( void )
{
	CC |= CC_E; 			/* HJB 980225: save entire state */
	PUSHWORD(pPC);
	PUSHWORD(pU);
	PUSHWORD(pY);
	PUSHWORD(pX);
	PUSHBYTE(DP);
	if ( MD & MD_EM )
	{
		PUSHBYTE(F);
		PUSHBYTE(E);
	}
	PUSHBYTE(B);
	PUSHBYTE(A);
	PUSHBYTE(CC);
	CC |= CC_IF | CC_II;	/* inhibit FIRQ and IRQ */
	PCD=RM16(0xfffa);
	CHANGE_PC;
}

/* $1130 BAND */

#define decodePB_tReg(n)	(((n) >> 6) & 0x03)
#define decodePB_src(n) 	(((n) >> 3) & 0x07)
#define decodePB_dst(n) 	(((n) >> 0) & 0x07)

static UINT8 dummy_byte;
static unsigned char *const regTable[4] = { &(CC), &(A), &(B), &dummy_byte };

static const UINT8 bitTable[] = { 0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80 };

HD6309_INLINE void band( void )
{
	UINT8		pb;
	UINT16		db;

	IMMBYTE(pb);

	DIRBYTE(db);

	if ( ( *(regTable[decodePB_tReg(pb)]) & bitTable[decodePB_dst(pb)] ) && ( db & bitTable[decodePB_src(pb)] ))
		*(regTable[decodePB_tReg(pb)]) |= bitTable[decodePB_dst(pb)];
	else
		*(regTable[decodePB_tReg(pb)]) &= (~bitTable[decodePB_dst(pb)]);
}

/* $1131 BIAND */

HD6309_INLINE void biand( void )
{
	UINT8		pb;
	UINT16		db;

	IMMBYTE(pb);

	DIRBYTE(db);

	if ( ( *(regTable[decodePB_tReg(pb)]) & bitTable[decodePB_dst(pb)] ) && ( (~db) & bitTable[decodePB_src(pb)] ))
		*(regTable[decodePB_tReg(pb)]) |= bitTable[decodePB_dst(pb)];
	else
		*(regTable[decodePB_tReg(pb)]) &= (~bitTable[decodePB_dst(pb)]);
}

/* $1132 BOR */

HD6309_INLINE void bor( void )
{
	UINT8		pb;
	UINT16		db;

	IMMBYTE(pb);

	DIRBYTE(db);

	if ( ( *(regTable[decodePB_tReg(pb)]) & bitTable[decodePB_dst(pb)] ) || ( db & bitTable[decodePB_src(pb)] ))
		*(regTable[decodePB_tReg(pb)]) |= bitTable[decodePB_dst(pb)];
	else
		*(regTable[decodePB_tReg(pb)]) &= (~bitTable[decodePB_dst(pb)]);
}

/* $1133 BIOR */

HD6309_INLINE void bior( void )
{
	UINT8		pb;
	UINT16		db;

	IMMBYTE(pb);

	DIRBYTE(db);

	if ( ( *(regTable[decodePB_tReg(pb)]) & bitTable[decodePB_dst(pb)] ) || ( (~db) & bitTable[decodePB_src(pb)] ))
		*(regTable[decodePB_tReg(pb)]) |= bitTable[decodePB_dst(pb)];
	else
		*(regTable[decodePB_tReg(pb)]) &= (~bitTable[decodePB_dst(pb)]);
}

/* $1134 BEOR */

HD6309_INLINE void beor( void )
{
	UINT8		pb;
	UINT16		db;
	UINT8		tReg, tMem;

	IMMBYTE(pb);

	DIRBYTE(db);

	tReg = *(regTable[decodePB_tReg(pb)]) & bitTable[decodePB_dst(pb)];
	tMem = db & bitTable[decodePB_src(pb)];

	if ( (tReg || tMem ) && !(tReg && tMem) )
		*(regTable[decodePB_tReg(pb)]) |= bitTable[decodePB_dst(pb)];
	else
		*(regTable[decodePB_tReg(pb)]) &= (~bitTable[decodePB_dst(pb)]);
}

/* $1135 BIEOR */

HD6309_INLINE void bieor( void )
{
	UINT8		pb;
	UINT16		db;
	UINT8		tReg, tMem;

	IMMBYTE(pb);

	DIRBYTE(db);

	tReg = *(regTable[decodePB_tReg(pb)]) & bitTable[decodePB_dst(pb)];
	tMem = (~db) & bitTable[decodePB_src(pb)];

	if ( (tReg || tMem ) && !(tReg && tMem) )
		*(regTable[decodePB_tReg(pb)]) |= bitTable[decodePB_dst(pb)];
	else
		*(regTable[decodePB_tReg(pb)]) &= (~bitTable[decodePB_dst(pb)]);
}

/* $1133 LDBT */

HD6309_INLINE void ldbt( void )
{
	UINT8		pb;
	UINT16		db;

	IMMBYTE(pb);

	DIRBYTE(db);

	if ( ( db & bitTable[decodePB_src(pb)] ) )
		*(regTable[decodePB_tReg(pb)]) |= bitTable[decodePB_dst(pb)];
	else
		*(regTable[decodePB_tReg(pb)]) &= (~bitTable[decodePB_dst(pb)]);
}

/* $1134 STBT */

HD6309_INLINE void stbt( void )
{
	UINT8		pb;
	UINT16		db;

	IMMBYTE(pb);

	DIRBYTE(db);

	if ( ( *(regTable[decodePB_tReg(pb)]) & bitTable[decodePB_dst(pb)] ) )
		WM( EAD, db | bitTable[decodePB_src(pb)] );
	else
		WM( EAD, db & (~bitTable[decodePB_src(pb)]) );
}

/* $103F SWI2 absolute indirect ----- */
HD6309_INLINE void swi2( void )
{
	CC |= CC_E; 			/* HJB 980225: save entire state */
	PUSHWORD(pPC);
	PUSHWORD(pU);
	PUSHWORD(pY);
	PUSHWORD(pX);
	PUSHBYTE(DP);
	if ( MD & MD_EM )
	{
		PUSHBYTE(F);
		PUSHBYTE(E);
	}
	PUSHBYTE(B);
	PUSHBYTE(A);
	PUSHBYTE(CC);
	PCD = RM16(0xfff4);
	CHANGE_PC;
}

/* $113F SWI3 absolute indirect ----- */
HD6309_INLINE void swi3( void )
{
	CC |= CC_E; 			/* HJB 980225: save entire state */
	PUSHWORD(pPC);
	PUSHWORD(pU);
	PUSHWORD(pY);
	PUSHWORD(pX);
	PUSHBYTE(DP);
	if ( MD & MD_EM )
	{
		PUSHBYTE(F);
		PUSHBYTE(E);
	}
	PUSHBYTE(B);
	PUSHBYTE(A);
	PUSHBYTE(CC);
	PCD = RM16(0xfff2);
	CHANGE_PC;
}

/* $40 NEGA inherent ?**** */
HD6309_INLINE void nega( void )
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
HD6309_INLINE void coma( void )
{
	A = ~A;
	CLR_NZV;
	SET_NZ8(A);
	SEC;
}

/* $44 LSRA inherent -0*-* */
HD6309_INLINE void lsra( void )
{
	CLR_NZC;
	CC |= (A & CC_C);
	A >>= 1;
	SET_Z8(A);
}

/* $45 ILLEGAL */

/* $46 RORA inherent -**-* */
HD6309_INLINE void rora( void )
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
HD6309_INLINE void asra( void )
{
	CLR_NZC;
	CC |= (A & CC_C);
	A = (A & 0x80) | (A >> 1);
	SET_NZ8(A);
}

/* $48 ASLA inherent ?**** */
HD6309_INLINE void asla( void )
{
	UINT16 r;
	r = A << 1;
	CLR_NZVC;
	SET_FLAGS8(A,A,r);
	A = r;
}

/* $49 ROLA inherent -**** */
HD6309_INLINE void rola( void )
{
	UINT16 t,r;
	t = A;
	r = (CC & CC_C) | (t<<1);
	CLR_NZVC; SET_FLAGS8(t,t,r);
	A = r;
}

/* $4A DECA inherent -***- */
HD6309_INLINE void deca( void )
{
	--A;
	CLR_NZV;
	SET_FLAGS8D(A);
}

/* $4B ILLEGAL */

/* $4C INCA inherent -***- */
HD6309_INLINE void inca( void )
{
	++A;
	CLR_NZV;
	SET_FLAGS8I(A);
}

/* $4D TSTA inherent -**0- */
HD6309_INLINE void tsta( void )
{
	CLR_NZV;
	SET_NZ8(A);
}

/* $4E ILLEGAL */

/* $4F CLRA inherent -0100 */
HD6309_INLINE void clra( void )
{
	A = 0;
	CLR_NZVC; SEZ;
}

/* $50 NEGB inherent ?**** */
HD6309_INLINE void negb( void )
{
	UINT16 r;
	r = -B;
	CLR_NZVC;
	SET_FLAGS8(0,B,r);
	B = r;
}

/* $1040 NEGD inherent ?**** */
HD6309_INLINE void negd( void )
{
	UINT32 r;
	r = -D;
	CLR_NZVC;
	SET_FLAGS16(0,D,r);
	D = r;
}

/* $51 ILLEGAL */

/* $52 ILLEGAL */

/* $53 COMB inherent -**01 */
HD6309_INLINE void comb( void )
{
	B = ~B;
	CLR_NZV;
	SET_NZ8(B);
	SEC;
}

/* $1143 COME inherent -**01 */
HD6309_INLINE void come( void )
{
	E = ~E;
	CLR_NZV;
	SET_NZ8(E);
	SEC;
}

/* $1153 COMF inherent -**01 */
HD6309_INLINE void comf( void )
{
	F = ~F;
	CLR_NZV;
	SET_NZ8(F);
	SEC;
}

/* $1043 COMD inherent -**01 */
HD6309_INLINE void comd( void )
{
	D = ~D;
	CLR_NZV;
	SET_NZ16(D);
	SEC;
}

/* $1053 COMW inherent -**01 */
HD6309_INLINE void comw( void )
{
	W = ~W;
	CLR_NZV;
	SET_NZ16(W);
	SEC;
}

/* $54 LSRB inherent -0*-* */
HD6309_INLINE void lsrb( void )
{
	CLR_NZC;
	CC |= (B & CC_C);
	B >>= 1;
	SET_Z8(B);
}

/* $1044 LSRD inherent -0*-* */
HD6309_INLINE void lsrd( void )
{
	CLR_NZC;
	CC |= (B & CC_C);
	D >>= 1;
	SET_Z16(D);
}

/* $1054 LSRW inherent -0*-* */
HD6309_INLINE void lsrw( void )
{
	CLR_NZC;
	CC |= (F & CC_C);
	W >>= 1;
	SET_Z16(W);
}

/* $55 ILLEGAL */

/* $56 RORB inherent -**-* */
HD6309_INLINE void rorb( void )
{
	UINT8 r;
	r = (CC & CC_C) << 7;
	CLR_NZC;
	CC |= (B & CC_C);
	r |= B >> 1;
	SET_NZ8(r);
	B = r;
}

/* $1046 RORD inherent -**-* */
HD6309_INLINE void rord( void )
{
	UINT16 r;
	r = (CC & CC_C) << 15;
	CLR_NZC;
	CC |= (D & CC_C);
	r |= D >> 1;
	SET_NZ16(r);
	D = r;
}

/* $1056 RORW inherent -**-* */
HD6309_INLINE void rorw( void )
{
	UINT16 r;
	r = (CC & CC_C) << 15;
	CLR_NZC;
	CC |= (W & CC_C);
	r |= W >> 1;
	SET_NZ16(r);
	W = r;
}

/* $57 ASRB inherent ?**-* */
HD6309_INLINE void asrb( void )
{
	CLR_NZC;
	CC |= (B & CC_C);
	B= (B & 0x80) | (B >> 1);
	SET_NZ8(B);
}

/* $1047 ASRD inherent ?**-* */
HD6309_INLINE void asrd( void )
{
	CLR_NZC;
	CC |= (D & CC_C);
	D= (D & 0x8000) | (D >> 1);
	SET_NZ16(D);
}

/* $58 ASLB inherent ?**** */
HD6309_INLINE void aslb( void )
{
	UINT16 r;
	r = B << 1;
	CLR_NZVC;
	SET_FLAGS8(B,B,r);
	B = r;
}

/* $1048 ASLD inherent ?**** */
HD6309_INLINE void asld( void )
{
	UINT32 r;
	r = D << 1;
	CLR_NZVC;
	SET_FLAGS16(D,D,r);
	D = r;
}

/* $59 ROLB inherent -**** */
HD6309_INLINE void rolb( void )
{
	UINT16 t,r;
	t = B;
	r = CC & CC_C;
	r |= t << 1;
	CLR_NZVC;
	SET_FLAGS8(t,t,r);
	B = r;
}

/* $1049 ROLD inherent -**** */
HD6309_INLINE void rold( void )
{
	UINT32 t,r;
	t = D;
	r = CC & CC_C;
	r |= t << 1;
	CLR_NZVC;
	SET_FLAGS16(t,t,r);
	D = r;
}

/* $1059 ROLW inherent -**** */
HD6309_INLINE void rolw( void )
{
	UINT32 t,r;
	t = W;
	r = CC & CC_C;
	r |= t << 1;
	CLR_NZVC;
	SET_FLAGS16(t,t,r);
	W = r;
}

/* $5A DECB inherent -***- */
HD6309_INLINE void decb( void )
{
	--B;
	CLR_NZV;
	SET_FLAGS8D(B);
}

/* $114a DECE inherent -***- */
HD6309_INLINE void dece( void )
{
	--E;
	CLR_NZV;
	SET_FLAGS8D(E);
}

/* $115a DECF inherent -***- */
HD6309_INLINE void decf( void )
{
	--F;
	CLR_NZV;
	SET_FLAGS8D(F);
}

/* $104a DECD inherent -***- */
HD6309_INLINE void decd( void )
{
	UINT32 r;
	r = D - 1;
	CLR_NZVC;
	SET_FLAGS16(D,D,r)
	D = r;
}

/* $105a DECW inherent -***- */
HD6309_INLINE void decw( void )
{
	UINT32 r;
	r = W - 1;
	CLR_NZVC;
	SET_FLAGS16(W,W,r)
	W = r;
}

/* $5B ILLEGAL */

/* $5C INCB inherent -***- */
HD6309_INLINE void incb( void )
{
	++B;
	CLR_NZV;
	SET_FLAGS8I(B);
}

/* $114c INCE inherent -***- */
HD6309_INLINE void ince( void )
{
	++E;
	CLR_NZV;
	SET_FLAGS8I(E);
}

/* $115c INCF inherent -***- */
HD6309_INLINE void incf( void )
{
	++F;
	CLR_NZV;
	SET_FLAGS8I(F);
}

/* $104c INCD inherent -***- */
HD6309_INLINE void incd( void )
{
	UINT32 r;
	r = D + 1;
	CLR_NZVC;
	SET_FLAGS16(D,D,r)
	D = r;
}

/* $105c INCW inherent -***- */
HD6309_INLINE void incw( void )
{
	UINT32 r;
	r = W + 1;
	CLR_NZVC;
	SET_FLAGS16(W,W,r)
	W = r;
}

/* $5D TSTB inherent -**0- */
HD6309_INLINE void tstb( void )
{
	CLR_NZV;
	SET_NZ8(B);
}

/* $104d TSTD inherent -**0- */
HD6309_INLINE void tstd( void )
{
	CLR_NZV;
	SET_NZ16(D);
}

/* $105d TSTW inherent -**0- */
HD6309_INLINE void tstw( void )
{
	CLR_NZV;
	SET_NZ16(W);
}

/* $114d TSTE inherent -**0- */
HD6309_INLINE void tste( void )
{
	CLR_NZV;
	SET_NZ8(E);
}

/* $115d TSTF inherent -**0- */
HD6309_INLINE void tstf( void )
{
	CLR_NZV;
	SET_NZ8(F);
}

/* $5E ILLEGAL */

/* $5F CLRB inherent -0100 */
HD6309_INLINE void clrb( void )
{
	B = 0;
	CLR_NZVC; SEZ;
}

/* $104f CLRD inherent -0100 */
HD6309_INLINE void clrd( void )
{
	D = 0;
	CLR_NZVC; SEZ;
}

/* $114f CLRE inherent -0100 */
HD6309_INLINE void clre( void )
{
	E = 0;
	CLR_NZVC; SEZ;
}

/* $115f CLRF inherent -0100 */
HD6309_INLINE void clrf( void )
{
	F = 0;
	CLR_NZVC; SEZ;
}

/* $105f CLRW inherent -0100 */
HD6309_INLINE void clrw( void )
{
	W = 0;
	CLR_NZVC; SEZ;
}

/* $60 NEG indexed ?**** */
HD6309_INLINE void neg_ix( void )
{
	UINT16 r,t;
	fetch_effective_address();
	t = RM(EAD);
	r=-t;
	CLR_NZVC;
	SET_FLAGS8(0,t,r);
	WM(EAD,r);
}

/* $61 OIM indexed */
HD6309_INLINE void oim_ix( void )
{
	UINT8	r,im;
	IMMBYTE(im);
	fetch_effective_address();
	r = im | RM(EAD);
	CLR_NZV;
	SET_NZ8(r);
	WM(EAD,r);
}

/* $62 AIM indexed */
HD6309_INLINE void aim_ix( void )
{
	UINT8	r,im;
	IMMBYTE(im);
	fetch_effective_address();
	r = im & RM(EAD);
	CLR_NZV;
	SET_NZ8(r);
	WM(EAD,r);
}

/* $63 COM indexed -**01 */
HD6309_INLINE void com_ix( void )
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
HD6309_INLINE void lsr_ix( void )
{
	UINT8 t;
	fetch_effective_address();
	t=RM(EAD);
	CLR_NZC;
	CC |= (t & CC_C);
	t>>=1; SET_Z8(t);
	WM(EAD,t);
}

/* $65 EIM indexed */
HD6309_INLINE void eim_ix( void )
{
	UINT8	r,im;
	IMMBYTE(im);
	fetch_effective_address();
	r = im ^ RM(EAD);
	CLR_NZV;
	SET_NZ8(r);
	WM(EAD,r);
}
/* $66 ROR indexed -**-* */
HD6309_INLINE void ror_ix( void )
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
HD6309_INLINE void asr_ix( void )
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
HD6309_INLINE void asl_ix( void )
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
HD6309_INLINE void rol_ix( void )
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
HD6309_INLINE void dec_ix( void )
{
	UINT8 t;
	fetch_effective_address();
	t = RM(EAD) - 1;
	CLR_NZV; SET_FLAGS8D(t);
	WM(EAD,t);
}

/* $6B TIM indexed */
HD6309_INLINE void tim_ix( void )
{
	UINT8	r,im,m;
	IMMBYTE(im);
	fetch_effective_address();
	m = RM(EAD);
	r = im & m;
	CLR_NZV;
	SET_NZ8(r);
}

/* $6C INC indexed -***- */
HD6309_INLINE void inc_ix( void )
{
	UINT8 t;
	fetch_effective_address();
	t = RM(EAD) + 1;
	CLR_NZV; SET_FLAGS8I(t);
	WM(EAD,t);
}

/* $6D TST indexed -**0- */
HD6309_INLINE void tst_ix( void )
{
	UINT8 t;
	fetch_effective_address();
	t = RM(EAD);
	CLR_NZV;
	SET_NZ8(t);
}

/* $6E JMP indexed ----- */
HD6309_INLINE void jmp_ix( void )
{
	fetch_effective_address();
	PCD = EAD;
	CHANGE_PC;
}

/* $6F CLR indexed -0100 */
HD6309_INLINE void clr_ix( void )
{
	UINT32 dummy;
	fetch_effective_address();
	dummy = RM(EAD);
	WM(EAD,0);
	CLR_NZVC; SEZ;
}

/* $70 NEG extended ?**** */
HD6309_INLINE void neg_ex( void )
{
	UINT16 r,t;
	EXTBYTE(t); r=-t;
	CLR_NZVC; SET_FLAGS8(0,t,r);
	WM(EAD,r);
}

/* $71 OIM extended */
HD6309_INLINE void oim_ex( void )
{
	UINT8	r,t,im;
	IMMBYTE(im);
	EXTBYTE(t);
	r = im | t;
	CLR_NZV;
	SET_NZ8(r);
	WM(EAD,r);
}

/* $72 AIM extended */
HD6309_INLINE void aim_ex( void )
{
	UINT8	r,t,im;
	IMMBYTE(im);
	EXTBYTE(t);
	r = im & t;
	CLR_NZV;
	SET_NZ8(r);
	WM(EAD,r);
}

/* $73 COM extended -**01 */
HD6309_INLINE void com_ex( void )
{
	UINT8 t;
	EXTBYTE(t); t = ~t;
	CLR_NZV; SET_NZ8(t); SEC;
	WM(EAD,t);
}

/* $74 LSR extended -0*-* */
HD6309_INLINE void lsr_ex( void )
{
	UINT8 t;
	EXTBYTE(t); CLR_NZC; CC |= (t & CC_C);
	t>>=1; SET_Z8(t);
	WM(EAD,t);
}

/* $75 EIM extended */
HD6309_INLINE void eim_ex( void )
{
	UINT8	r,t,im;
	IMMBYTE(im);
	EXTBYTE(t);
	r = im ^ t;
	CLR_NZV;
	SET_NZ8(r);
	WM(EAD,r);
}

/* $76 ROR extended -**-* */
HD6309_INLINE void ror_ex( void )
{
	UINT8 t,r;
	EXTBYTE(t); r=(CC & CC_C) << 7;
	CLR_NZC; CC |= (t & CC_C);
	r |= t>>1; SET_NZ8(r);
	WM(EAD,r);
}

/* $77 ASR extended ?**-* */
HD6309_INLINE void asr_ex( void )
{
	UINT8 t;
	EXTBYTE(t); CLR_NZC; CC |= (t & CC_C);
	t=(t&0x80)|(t>>1);
	SET_NZ8(t);
	WM(EAD,t);
}

/* $78 ASL extended ?**** */
HD6309_INLINE void asl_ex( void )
{
	UINT16 t,r;
	EXTBYTE(t); r=t<<1;
	CLR_NZVC; SET_FLAGS8(t,t,r);
	WM(EAD,r);
}

/* $79 ROL extended -**** */
HD6309_INLINE void rol_ex( void )
{
	UINT16 t,r;
	EXTBYTE(t); r = (CC & CC_C) | (t << 1);
	CLR_NZVC; SET_FLAGS8(t,t,r);
	WM(EAD,r);
}

/* $7A DEC extended -***- */
HD6309_INLINE void dec_ex( void )
{
	UINT8 t;
	EXTBYTE(t); --t;
	CLR_NZV; SET_FLAGS8D(t);
	WM(EAD,t);
}

/* $7B TIM extended */
HD6309_INLINE void tim_ex( void )
{
	UINT8	r,t,im;
	IMMBYTE(im);
	EXTBYTE(t);
	r = im & t;
	CLR_NZV;
	SET_NZ8(r);
}

/* $7C INC extended -***- */
HD6309_INLINE void inc_ex( void )
{
	UINT8 t;
	EXTBYTE(t); ++t;
	CLR_NZV; SET_FLAGS8I(t);
	WM(EAD,t);
}

/* $7D TST extended -**0- */
HD6309_INLINE void tst_ex( void )
{
	UINT8 t;
	EXTBYTE(t); CLR_NZV; SET_NZ8(t);
}

/* $7E JMP extended ----- */
HD6309_INLINE void jmp_ex( void )
{
	EXTENDED;
	PCD = EAD;
	CHANGE_PC;
}

/* $7F CLR extended -0100 */
HD6309_INLINE void clr_ex( void )
{
	UINT32 dummy;
	EXTENDED;
	dummy = RM(EAD);
	WM(EAD,0);
	CLR_NZVC; SEZ;
}

/* $80 SUBA immediate ?**** */
HD6309_INLINE void suba_im( void )
{
	UINT16 t,r;
	IMMBYTE(t);
	r = A - t;
	CLR_NZVC;
	SET_FLAGS8(A,t,r);
	A = r;
}

/* $81 CMPA immediate ?**** */
HD6309_INLINE void cmpa_im( void )
{
	UINT16	  t,r;
	IMMBYTE(t);
	r = A - t;
	CLR_NZVC;
	SET_FLAGS8(A,t,r);
}

/* $82 SBCA immediate ?**** */
HD6309_INLINE void sbca_im( void )
{
	UINT16	  t,r;
	IMMBYTE(t);
	r = A - t - (CC & CC_C);
	CLR_NZVC;
	SET_FLAGS8(A,t,r);
	A = r;
}

/* $83 SUBD (CMPD CMPU) immediate -**** */
HD6309_INLINE void subd_im( void )
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

/* $1080 SUBW immediate -**** */
HD6309_INLINE void subw_im( void )
{
	UINT32 r,d;
	PAIR b;
	IMMWORD(b);
	d = W;
	r = d - b.d;
	CLR_NZVC;
	SET_FLAGS16(d,b.d,r);
	W = r;
}

/* $1083 CMPD immediate -**** */
HD6309_INLINE void cmpd_im( void )
{
	UINT32 r,d;
	PAIR b;
	IMMWORD(b);
	d = D;
	r = d - b.d;
	CLR_NZVC;
	SET_FLAGS16(d,b.d,r);
}

/* $1081 CMPW immediate -**** */
HD6309_INLINE void cmpw_im( void )
{
	UINT32 r,d;
	PAIR b;
	IMMWORD(b);
	d = W;
	r = d - b.d;
	CLR_NZVC;
	SET_FLAGS16(d,b.d,r);
}

/* $1183 CMPU immediate -**** */
HD6309_INLINE void cmpu_im( void )
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
HD6309_INLINE void anda_im( void )
{
	UINT8 t;
	IMMBYTE(t);
	A &= t;
	CLR_NZV;
	SET_NZ8(A);
}

/* $85 BITA immediate -**0- */
HD6309_INLINE void bita_im( void )
{
	UINT8 t,r;
	IMMBYTE(t);
	r = A & t;
	CLR_NZV;
	SET_NZ8(r);
}

/* $86 LDA immediate -**0- */
HD6309_INLINE void lda_im( void )
{
	IMMBYTE(A);
	CLR_NZV;
	SET_NZ8(A);
}

/* $88 EORA immediate -**0- */
HD6309_INLINE void eora_im( void )
{
	UINT8 t;
	IMMBYTE(t);
	A ^= t;
	CLR_NZV;
	SET_NZ8(A);
}

/* $89 ADCA immediate ***** */
HD6309_INLINE void adca_im( void )
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
HD6309_INLINE void ora_im( void )
{
	UINT8 t;
	IMMBYTE(t);
	A |= t;
	CLR_NZV;
	SET_NZ8(A);
}

/* $8B ADDA immediate ***** */
HD6309_INLINE void adda_im( void )
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
HD6309_INLINE void cmpx_im( void )
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
HD6309_INLINE void cmpy_im( void )
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
HD6309_INLINE void cmps_im( void )
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
HD6309_INLINE void bsr( void )
{
	UINT8 t;
	IMMBYTE(t);
	PUSHWORD(pPC);
	PC += SIGNED(t);
	CHANGE_PC;
}

/* $8E LDX (LDY) immediate -**0- */
HD6309_INLINE void ldx_im( void )
{
	IMMWORD(pX);
	CLR_NZV;
	SET_NZ16(X);
}

/* $CD LDQ immediate -**0- */
HD6309_INLINE void ldq_im( void )
{
	PAIR	q;

	IMMLONG(q);
	D = q.w.h;
	W = q.w.l;
	CLR_NZV;
	SET_N8(A);
	SET_Z(q.d);
}

/* $108E LDY immediate -**0- */
HD6309_INLINE void ldy_im( void )
{
	IMMWORD(pY);
	CLR_NZV;
	SET_NZ16(Y);
}

/* $118f MULD immediate */
HD6309_INLINE void muld_im( void )
{
	PAIR t, q;

	IMMWORD( t );
	q.d = (INT16) D * (INT16)t.w.l;
	D = q.w.h;
	W = q.w.l;
	CLR_NZVC;
	SET_NZ16(D);
}

/* $118d DIVD immediate */
HD6309_INLINE void divd_im( void )
{
	UINT8   t;
	INT16   v, oldD;

	IMMBYTE( t );

	if( t != 0 )
	{
		oldD = D;
		v = (INT16) D / (INT8) t;
		A = (INT16) D % (INT8) t;
		B = v;

		CLR_NZVC;
		SET_NZ8(B);

		if( B & 0x01 )
			SEC;

		if( (INT16)D < 0 )
			SEN;

		if ( (v > 127) || (v < -128) ) /* soft overflow */
		{
			SEV;

			if( (v > 255) || (v < -256) ) /* hard overflow - division is aborted */
			{
				SET_NZ16( oldD );
				D = abs( oldD );
			}
		}
	}
	else
	{
		hd6309_ICount -= 8;
		DZError();
	}
}

/* $118e DIVQ immediate */
HD6309_INLINE void divq_im( void )
{
	PAIR	t,q, oldQ;
	INT32	v;

	IMMWORD( t );

	q.w.h = D;
	q.w.l = W;

	if( t.w.l != 0 )
	{
		oldQ = q;

		v = (INT32) q.d / (INT16) t.w.l;
		D = (INT32) q.d % (INT16) t.w.l;
		W = v;

		CLR_NZVC;
		SET_NZ16(W);

		if( W & 0x0001 )
			SEC;

		if ( (v > 32768) || (v < -32767) ) /* soft overflow */
		{
			SEV;

			if( (v > 65536 ) || (v < -65535 ) ) /* hard overflow - division is aborted */
			{
				if( (INT32)oldQ.d < 0 )
					SEN;
				else if( oldQ.d == 0 )
					SEZ;

				t.w.l = abs( t.w.l );
				D = oldQ.w.h;
				W = oldQ.w.l;
			}
		}
	}
	else
		DZError();
}

/* $90 SUBA direct ?**** */
HD6309_INLINE void suba_di( void )
{
	UINT16	  t,r;
	DIRBYTE(t);
	r = A - t;
	CLR_NZVC;
	SET_FLAGS8(A,t,r);
	A = r;
}

/* $91 CMPA direct ?**** */
HD6309_INLINE void cmpa_di( void )
{
	UINT16	  t,r;
	DIRBYTE(t);
	r = A - t;
	CLR_NZVC;
	SET_FLAGS8(A,t,r);
}

/* $92 SBCA direct ?**** */
HD6309_INLINE void sbca_di( void )
{
	UINT16	  t,r;
	DIRBYTE(t);
	r = A - t - (CC & CC_C);
	CLR_NZVC;
	SET_FLAGS8(A,t,r);
	A = r;
}

/* $93 SUBD (CMPD CMPU) direct -**** */
HD6309_INLINE void subd_di( void )
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

/* $1090 SUBW direct -**** */
HD6309_INLINE void subw_di( void )
{
	UINT32 r,d;
	PAIR b;
	DIRWORD(b);
	d = W;
	r = d - b.d;
	CLR_NZVC;
	SET_FLAGS16(d,b.d,r);
	W = r;
}

/* $1093 CMPD direct -**** */
HD6309_INLINE void cmpd_di( void )
{
	UINT32 r,d;
	PAIR b;
	DIRWORD(b);
	d = D;
	r = d - b.d;
	CLR_NZVC;
	SET_FLAGS16(d,b.d,r);
}

/* $1091 CMPW direct -**** */
HD6309_INLINE void cmpw_di( void )
{
	UINT32 r,d;
	PAIR b;
	DIRWORD(b);
	d = W;
	r = d - b.d;
	CLR_NZVC;
	SET_FLAGS16(d,b.d,r);
}

/* $1193 CMPU direct -**** */
HD6309_INLINE void cmpu_di( void )
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
HD6309_INLINE void anda_di( void )
{
	UINT8 t;
	DIRBYTE(t);
	A &= t;
	CLR_NZV;
	SET_NZ8(A);
}

/* $95 BITA direct -**0- */
HD6309_INLINE void bita_di( void )
{
	UINT8 t,r;
	DIRBYTE(t);
	r = A & t;
	CLR_NZV;
	SET_NZ8(r);
}

/* $96 LDA direct -**0- */
HD6309_INLINE void lda_di( void )
{
	DIRBYTE(A);
	CLR_NZV;
	SET_NZ8(A);
}

/* $97 STA direct -**0- */
HD6309_INLINE void sta_di( void )
{
	CLR_NZV;
	SET_NZ8(A);
	DIRECT;
	WM(EAD,A);
}

/* $98 EORA direct -**0- */
HD6309_INLINE void eora_di( void )
{
	UINT8 t;
	DIRBYTE(t);
	A ^= t;
	CLR_NZV;
	SET_NZ8(A);
}

/* $99 ADCA direct ***** */
HD6309_INLINE void adca_di( void )
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
HD6309_INLINE void ora_di( void )
{
	UINT8 t;
	DIRBYTE(t);
	A |= t;
	CLR_NZV;
	SET_NZ8(A);
}

/* $9B ADDA direct ***** */
HD6309_INLINE void adda_di( void )
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
HD6309_INLINE void cmpx_di( void )
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
HD6309_INLINE void cmpy_di( void )
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
HD6309_INLINE void cmps_di( void )
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
HD6309_INLINE void jsr_di( void )
{
	DIRECT;
	PUSHWORD(pPC);
	PCD = EAD;
	CHANGE_PC;
}

/* $9E LDX (LDY) direct -**0- */
HD6309_INLINE void ldx_di( void )
{
	DIRWORD(pX);
	CLR_NZV;
	SET_NZ16(X);
}

/* $119f MULD direct -**0- */
HD6309_INLINE void muld_di( void )
{
	PAIR	t,q;

	DIRWORD(t);
	q.d = (INT16) D * (INT16)t.w.l;

	D = q.w.h;
	W = q.w.l;
	CLR_NZVC;
	SET_NZ16(D);
}

/* $119d DIVD direct -**0- */
HD6309_INLINE void divd_di( void )
{
	UINT8	t;
	INT16   v, oldD;

	DIRBYTE(t);

	if( t != 0 )
	{
		oldD = D;
		v = (INT16) D / (INT8) t;
		A = (INT16) D % (INT8) t;
		B = v;

		CLR_NZVC;
		SET_NZ8(B);

		if( B & 0x01 )
			SEC;

		if( (INT16)D < 0 )
			SEN;

		if ( (v > 127) || (v < -128) ) /* soft overflow */
		{
			SEV;

			if( (v > 255) || (v < -256) ) /* hard overflow - division is aborted */
			{
				SET_NZ16( oldD );
				D = abs( oldD );
			}
		}
	}
	else
	{
		hd6309_ICount -= 8;
		DZError();
	}
}

/* $119e DIVQ direct -**0- */
HD6309_INLINE void divq_di( void )
{
	PAIR	t,q, oldQ;
	INT32	v;

	DIRWORD(t);

	q.w.h = D;
	q.w.l = W;

	if( t.w.l != 0 )
	{
		oldQ = q;

		v = (INT32) q.d / (INT16) t.w.l;
		D = (INT32) q.d % (INT16) t.w.l;
		W = v;

		CLR_NZVC;
		SET_NZ16(W);

		if( W & 0x0001 )
			SEC;

		if ( (v > 32767) || (v < -32768) ) /* soft overflow */
		{
			SEV;

			if( (v > 65535 ) || (v < -65536 ) ) /* hard overflow - division is aborted */
			{
				if( (INT32)oldQ.d < 0 )
					SEN;
				else if( oldQ.d == 0 )
					SEZ;

				t.w.l = abs( t.w.l );
				D = oldQ.w.h;
				W = oldQ.w.l;
			}
		}
	}
	else
		DZError();
}
/* $10dc LDQ direct -**0- */
HD6309_INLINE void ldq_di( void )
{
	PAIR	q;

	DIRLONG(q);
	D = q.w.h;
	W = q.w.l;
	CLR_NZV;
	SET_N8(A);
	SET_Z(q.d);
}

/* $109E LDY direct -**0- */
HD6309_INLINE void ldy_di( void )
{
	DIRWORD(pY);
	CLR_NZV;
	SET_NZ16(Y);
}

/* $9F STX (STY) direct -**0- */
HD6309_INLINE void stx_di( void )
{
	CLR_NZV;
	SET_NZ16(X);
	DIRECT;
	WM16(EAD,&pX);
}

/* $10dd STQ direct -**0- */
HD6309_INLINE void stq_di( void )
{
	PAIR	q;

	q.w.h = D;
	q.w.l = W;
	DIRECT;
	WM32(EAD,&q);
	CLR_NZV;
	SET_N8(A);
	SET_Z(q.d);
}

/* $109F STY direct -**0- */
HD6309_INLINE void sty_di( void )
{
	CLR_NZV;
	SET_NZ16(Y);
	DIRECT;
	WM16(EAD,&pY);
}

/* $a0 SUBA indexed ?**** */
HD6309_INLINE void suba_ix( void )
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
HD6309_INLINE void cmpa_ix( void )
{
	UINT16 t,r;
	fetch_effective_address();
	t = RM(EAD);
	r = A - t;
	CLR_NZVC;
	SET_FLAGS8(A,t,r);
}

/* $a2 SBCA indexed ?**** */
HD6309_INLINE void sbca_ix( void )
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
HD6309_INLINE void subd_ix( void )
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

/* $10a0 SUBW indexed -**** */
HD6309_INLINE void subw_ix( void )
{
	UINT32 r,d;
	PAIR b;
	fetch_effective_address();
	b.d=RM16(EAD);
	d = W;
	r = d - b.d;
	CLR_NZVC;
	SET_FLAGS16(d,b.d,r);
	W = r;
}

/* $10a3 CMPD indexed -**** */
HD6309_INLINE void cmpd_ix( void )
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

/* $10a1 CMPW indexed -**** */
HD6309_INLINE void cmpw_ix( void )
{
	UINT32 r,d;
	PAIR b;
	fetch_effective_address();
	b.d=RM16(EAD);
	d = W;
	r = d - b.d;
	CLR_NZVC;
	SET_FLAGS16(d,b.d,r);
}

/* $11a3 CMPU indexed -**** */
HD6309_INLINE void cmpu_ix( void )
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
HD6309_INLINE void anda_ix( void )
{
	fetch_effective_address();
	A &= RM(EAD);
	CLR_NZV;
	SET_NZ8(A);
}

/* $a5 BITA indexed -**0- */
HD6309_INLINE void bita_ix( void )
{
	UINT8 r;
	fetch_effective_address();
	r = A & RM(EAD);
	CLR_NZV;
	SET_NZ8(r);
}

/* $a6 LDA indexed -**0- */
HD6309_INLINE void lda_ix( void )
{
	fetch_effective_address();
	A = RM(EAD);
	CLR_NZV;
	SET_NZ8(A);
}

/* $a7 STA indexed -**0- */
HD6309_INLINE void sta_ix( void )
{
	fetch_effective_address();
	CLR_NZV;
	SET_NZ8(A);
	WM(EAD,A);
}

/* $a8 EORA indexed -**0- */
HD6309_INLINE void eora_ix( void )
{
	fetch_effective_address();
	A ^= RM(EAD);
	CLR_NZV;
	SET_NZ8(A);
}

/* $a9 ADCA indexed ***** */
HD6309_INLINE void adca_ix( void )
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
HD6309_INLINE void ora_ix( void )
{
	fetch_effective_address();
	A |= RM(EAD);
	CLR_NZV;
	SET_NZ8(A);
}

/* $aB ADDA indexed ***** */
HD6309_INLINE void adda_ix( void )
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
HD6309_INLINE void cmpx_ix( void )
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
HD6309_INLINE void cmpy_ix( void )
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
HD6309_INLINE void cmps_ix( void )
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
HD6309_INLINE void jsr_ix( void )
{
	fetch_effective_address();
	PUSHWORD(pPC);
	PCD = EAD;
	CHANGE_PC;
}

/* $aE LDX (LDY) indexed -**0- */
HD6309_INLINE void ldx_ix( void )
{
	fetch_effective_address();
	X=RM16(EAD);
	CLR_NZV;
	SET_NZ16(X);
}

/* $11af MULD indexed -**0- */
HD6309_INLINE void muld_ix( void )
{
	PAIR	q;
	UINT16	t;

	fetch_effective_address();
	t=RM16(EAD);
	q.d = (INT16) D * (INT16)t;

	D = q.w.h;
	W = q.w.l;
	CLR_NZVC;
	SET_NZ16(D);
}

/* $11ad DIVD indexed -**0- */
HD6309_INLINE void divd_ix( void )
{
	UINT8	t;
	INT16   v, oldD;

	fetch_effective_address();
	t=RM(EAD);

	if( t != 0 )
	{
		oldD = D;
		v = (INT16) D / (INT8) t;
		A = (INT16) D % (INT8) t;
		B = v;

		CLR_NZVC;
		SET_NZ8(B);

		if( B & 0x01 )
			SEC;

		if( (INT16)D < 0 )
			SEN;

		if ( (v > 127) || (v < -128) ) /* soft overflow */
		{
			SEV;

			if( (v > 255) || (v < -256) ) /* hard overflow - division is aborted */
			{
				SET_NZ16( oldD );
				D = abs( oldD );
			}
		}
	}
	else
	{
		hd6309_ICount -= 8;
		DZError();
	}
}

/* $11ae DIVQ indexed -**0- */
HD6309_INLINE void divq_ix( void )
{
	PAIR	t,q, oldQ;
	INT32	v;

	fetch_effective_address();
	t.w.l=RM16(EAD);

	q.w.h = D;
	q.w.l = W;

	if( t.w.l != 0 )
	{
		oldQ = q;

		v = (INT32) q.d / (INT16) t.w.l;
		D = (INT32) q.d % (INT16) t.w.l;
		W = v;

		CLR_NZVC;
		SET_NZ16(W);

		if( W & 0x0001 )
			SEC;

		if ( (v > 32767) || (v < -32768) ) /* soft overflow */
		{
			SEV;

			if( (v > 65535 ) || (v < -65536 ) ) /* hard overflow - division is aborted */
			{
				if( (INT32)oldQ.d < 0 )
					SEN;
				else if( oldQ.d == 0 )
					SEZ;

				t.w.l = abs( t.w.l );
				D = oldQ.w.h;
				W = oldQ.w.l;
			}
		}
	}
	else
		DZError();
}

/* $10ec LDQ indexed -**0- */
HD6309_INLINE void ldq_ix( void )
{
	PAIR	q;

	fetch_effective_address();
	q.d=RM32(EAD);
	D = q.w.h;
	W = q.w.l;
	CLR_NZV;
	SET_N8(A);
	SET_Z(q.d);
}

/* $10aE LDY indexed -**0- */
HD6309_INLINE void ldy_ix( void )
{
	fetch_effective_address();
	Y=RM16(EAD);
	CLR_NZV;
	SET_NZ16(Y);
}

/* $aF STX (STY) indexed -**0- */
HD6309_INLINE void stx_ix( void )
{
	fetch_effective_address();
	CLR_NZV;
	SET_NZ16(X);
	WM16(EAD,&pX);
}

/* $10ed STQ indexed -**0- */
HD6309_INLINE void stq_ix( void )
{
	PAIR	q;

	q.w.h = D;
	q.w.l = W;
	fetch_effective_address();
	WM32(EAD,&q);
	CLR_NZV;
	SET_N8(A);
	SET_Z(q.d);
}

/* $10aF STY indexed -**0- */
HD6309_INLINE void sty_ix( void )
{
	fetch_effective_address();
	CLR_NZV;
	SET_NZ16(Y);
	WM16(EAD,&pY);
}

/* $b0 SUBA extended ?**** */
HD6309_INLINE void suba_ex( void )
{
	UINT16	  t,r;
	EXTBYTE(t);
	r = A - t;
	CLR_NZVC;
	SET_FLAGS8(A,t,r);
	A = r;
}

/* $b1 CMPA extended ?**** */
HD6309_INLINE void cmpa_ex( void )
{
	UINT16	  t,r;
	EXTBYTE(t);
	r = A - t;
	CLR_NZVC;
	SET_FLAGS8(A,t,r);
}

/* $b2 SBCA extended ?**** */
HD6309_INLINE void sbca_ex( void )
{
	UINT16	  t,r;
	EXTBYTE(t);
	r = A - t - (CC & CC_C);
	CLR_NZVC;
	SET_FLAGS8(A,t,r);
	A = r;
}

/* $b3 SUBD (CMPD CMPU) extended -**** */
HD6309_INLINE void subd_ex( void )
{
	UINT32 r,d;
	PAIR b = {{0,0,0,0}};
	EXTWORD(b);
	d = D;
	r = d - b.d;
	CLR_NZVC;
	SET_FLAGS16(d,b.d,r);
	D = r;
}

/* $10b0 SUBW extended -**** */
HD6309_INLINE void subw_ex( void )
{
	UINT32 r,d;
	PAIR b = {{0,0,0,0}};
	EXTWORD(b);
	d = W;
	r = d - b.d;
	CLR_NZVC;
	SET_FLAGS16(d,b.d,r);
	W = r;
}

/* $10b3 CMPD extended -**** */
HD6309_INLINE void cmpd_ex( void )
{
	UINT32 r,d;
	PAIR b = {{0,0,0,0}};
	EXTWORD(b);
	d = D;
	r = d - b.d;
	CLR_NZVC;
	SET_FLAGS16(d,b.d,r);
}

/* $10b1 CMPW extended -**** */
HD6309_INLINE void cmpw_ex( void )
{
	UINT32 r,d;
	PAIR b = {{0,0,0,0}};
	EXTWORD(b);
	d = W;
	r = d - b.d;
	CLR_NZVC;
	SET_FLAGS16(d,b.d,r);
}

/* $11b3 CMPU extended -**** */
HD6309_INLINE void cmpu_ex( void )
{
	UINT32 r,d;
	PAIR b = {{0,0,0,0}};
	EXTWORD(b);
	d = U;
	r = d - b.d;
	CLR_NZVC;
	SET_FLAGS16(d,b.d,r);
}

/* $b4 ANDA extended -**0- */
HD6309_INLINE void anda_ex( void )
{
	UINT8 t;
	EXTBYTE(t);
	A &= t;
	CLR_NZV;
	SET_NZ8(A);
}

/* $b5 BITA extended -**0- */
HD6309_INLINE void bita_ex( void )
{
	UINT8 t,r;
	EXTBYTE(t);
	r = A & t;
	CLR_NZV; SET_NZ8(r);
}

/* $b6 LDA extended -**0- */
HD6309_INLINE void lda_ex( void )
{
	EXTBYTE(A);
	CLR_NZV;
	SET_NZ8(A);
}

/* $b7 STA extended -**0- */
HD6309_INLINE void sta_ex( void )
{
	CLR_NZV;
	SET_NZ8(A);
	EXTENDED;
	WM(EAD,A);
}

/* $b8 EORA extended -**0- */
HD6309_INLINE void eora_ex( void )
{
	UINT8 t;
	EXTBYTE(t);
	A ^= t;
	CLR_NZV;
	SET_NZ8(A);
}

/* $b9 ADCA extended ***** */
HD6309_INLINE void adca_ex( void )
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
HD6309_INLINE void ora_ex( void )
{
	UINT8 t;
	EXTBYTE(t);
	A |= t;
	CLR_NZV;
	SET_NZ8(A);
}

/* $bB ADDA extended ***** */
HD6309_INLINE void adda_ex( void )
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
HD6309_INLINE void cmpx_ex( void )
{
	UINT32 r,d;
	PAIR b = {{0,0,0,0}};
	EXTWORD(b);
	d = X;
	r = d - b.d;
	CLR_NZVC;
	SET_FLAGS16(d,b.d,r);
}

/* $10bC CMPY extended -**** */
HD6309_INLINE void cmpy_ex( void )
{
	UINT32 r,d;
	PAIR b = {{0,0,0,0}};
	EXTWORD(b);
	d = Y;
	r = d - b.d;
	CLR_NZVC;
	SET_FLAGS16(d,b.d,r);
}

/* $11bC CMPS extended -**** */
HD6309_INLINE void cmps_ex( void )
{
	UINT32 r,d;
	PAIR b = {{0,0,0,0}};
	EXTWORD(b);
	d = S;
	r = d - b.d;
	CLR_NZVC;
	SET_FLAGS16(d,b.d,r);
}

/* $bD JSR extended ----- */
HD6309_INLINE void jsr_ex( void )
{
	EXTENDED;
	PUSHWORD(pPC);
	PCD = EAD;
	CHANGE_PC;
}

/* $bE LDX (LDY) extended -**0- */
HD6309_INLINE void ldx_ex( void )
{
	EXTWORD(pX);
	CLR_NZV;
	SET_NZ16(X);
}

/* $11bf MULD extended -**0- */
HD6309_INLINE void muld_ex( void )
{
	PAIR	t, q;

	EXTWORD(t);
	q.d = (INT16) D * (INT16)t.w.l;

	D = q.w.h;
	W = q.w.l;
	CLR_NZVC;
	SET_NZ16(D);
}

/* $11bd DIVD extended -**0- */
HD6309_INLINE void divd_ex( void )
{
	UINT8	t;
	INT16   v, oldD;

	EXTBYTE(t);

	if( t != 0 )
	{
		oldD = D;
		v = (INT16) D / (INT8) t;
		A = (INT16) D % (INT8) t;
		B = v;

		CLR_NZVC;
		SET_NZ8(B);

		if( B & 0x01 )
			SEC;

		if( (INT16)D < 0 )
			SEN;

		if ( (v > 127) || (v < -128) ) /* soft overflow */
		{
			SEV;

			if( (v > 255) || (v < -256) ) /* hard overflow - division is aborted */
			{
				SET_NZ16( oldD );
				D = abs( oldD );
			}
		}
	}
	else
	{
		hd6309_ICount -= 8;
		DZError();
	}
}

/* $11be DIVQ extended -**0- */
HD6309_INLINE void divq_ex( void )
{
	PAIR	t,q, oldQ;
	INT32	v;

	EXTWORD(t);

	q.w.h = D;
	q.w.l = W;

	if( t.w.l != 0 )
	{
		oldQ = q;

		v = (INT32) q.d / (INT16) t.w.l;
		D = (INT32) q.d % (INT16) t.w.l;
		W = v;

		CLR_NZVC;
		SET_NZ16(W);

		if( W & 0x0001 )
			SEC;

		if ( (v > 32767) || (v < -32768) ) /* soft overflow */
		{
			SEV;

			if( (v > 65535 ) || (v < -65536 ) ) /* hard overflow - division is aborted */
			{
				if( (INT32)oldQ.d < 0 )
					SEN;
				else if( oldQ.d == 0 )
					SEZ;

				t.w.l = abs( t.w.l );
				D = oldQ.w.h;
				W = oldQ.w.l;
			}
		}
	}
	else
		DZError();
}

/* $10fc LDQ extended -**0- */
HD6309_INLINE void ldq_ex( void )
{
	PAIR	q;

	EXTLONG(q);
	D = q.w.h;
	W = q.w.l;
	CLR_NZV;
	SET_N8(A);
	SET_Z(q.d);
}

/* $10bE LDY extended -**0- */
HD6309_INLINE void ldy_ex( void )
{
	EXTWORD(pY);
	CLR_NZV;
	SET_NZ16(Y);
}

/* $bF STX (STY) extended -**0- */
HD6309_INLINE void stx_ex( void )
{
	CLR_NZV;
	SET_NZ16(X);
	EXTENDED;
	WM16(EAD,&pX);
}

/* $10fd STQ extended -**0- */
HD6309_INLINE void stq_ex( void )
{
	PAIR	q;

	q.w.h = D;
	q.w.l = W;
	EXTENDED;
	WM32(EAD,&q);
	CLR_NZV;
	SET_N8(A);
	SET_Z(q.d);
}

/* $10bF STY extended -**0- */
HD6309_INLINE void sty_ex( void )
{
	CLR_NZV;
	SET_NZ16(Y);
	EXTENDED;
	WM16(EAD,&pY);
}

/* $c0 SUBB immediate ?**** */
HD6309_INLINE void subb_im( void )
{
	UINT16	  t,r;
	IMMBYTE(t);
	r = B - t;
	CLR_NZVC;
	SET_FLAGS8(B,t,r);
	B = r;
}

/* $1180 SUBE immediate ?**** */
HD6309_INLINE void sube_im( void )
{
	UINT16	  t,r;
	IMMBYTE(t);
	r = E - t;
	CLR_NZVC;
	SET_FLAGS8(E,t,r);
	E = r;
}

/* $11C0 SUBF immediate ?**** */
HD6309_INLINE void subf_im( void )
{
	UINT16	  t,r;
	IMMBYTE(t);
	r = F - t;
	CLR_NZVC;
	SET_FLAGS8(F,t,r);
	F = r;
}

/* $c1 CMPB immediate ?**** */
HD6309_INLINE void cmpb_im( void )
{
	UINT16	  t,r;
	IMMBYTE(t);
	r = B - t;
	CLR_NZVC; SET_FLAGS8(B,t,r);
}

/* $1181 CMPE immediate ?**** */
HD6309_INLINE void cmpe_im( void )
{
	UINT16	  t,r;
	IMMBYTE(t);
	r = E - t;
	CLR_NZVC; SET_FLAGS8(E,t,r);
}

/* $11C1 CMPF immediate ?**** */
HD6309_INLINE void cmpf_im( void )
{
	UINT16	  t,r;
	IMMBYTE(t);
	r = F - t;
	CLR_NZVC; SET_FLAGS8(F,t,r);
}

/* $c2 SBCB immediate ?**** */
HD6309_INLINE void sbcb_im( void )
{
	UINT16	  t,r;
	IMMBYTE(t);
	r = B - t - (CC & CC_C);
	CLR_NZVC;
	SET_FLAGS8(B,t,r);
	B = r;
}

/* $1082 SBCD immediate ?**** */
HD6309_INLINE void sbcd_im( void )
{
	PAIR	t;
	UINT32	 r;
	IMMWORD(t);
	r = D - t.w.l - (CC & CC_C);
	CLR_NZVC;
	SET_FLAGS16(D,t.w.l,r);
	D = r;
}

/* $c3 ADDD immediate -**** */
HD6309_INLINE void addd_im( void )
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

/* $108b ADDW immediate -**** */
HD6309_INLINE void addw_im( void )
{
	UINT32 r,d;
	PAIR b;
	IMMWORD(b);
	d = W;
	r = d + b.d;
	CLR_NZVC;
	SET_FLAGS16(d,b.d,r);
	W = r;
}

/* $118b ADDE immediate -**** */
HD6309_INLINE void adde_im( void )
{
	UINT16 t,r;
	IMMBYTE(t);
	r = E + t;
	CLR_HNZVC;
	SET_FLAGS8(E,t,r);
	SET_H(E,t,r);
	E = r;
}

/* $11Cb ADDF immediate -**** */
HD6309_INLINE void addf_im( void )
{
	UINT16 t,r;
	IMMBYTE(t);
	r = F + t;
	CLR_HNZVC;
	SET_FLAGS8(F,t,r);
	SET_H(F,t,r);
	F = r;
}

/* $c4 ANDB immediate -**0- */
HD6309_INLINE void andb_im( void )
{
	UINT8 t;
	IMMBYTE(t);
	B &= t;
	CLR_NZV;
	SET_NZ8(B);
}

/* $1084 ANDD immediate -**0- */
HD6309_INLINE void andd_im( void )
{
	PAIR t;
	IMMWORD(t);
	D &= t.w.l;
	CLR_NZV;
	SET_NZ16(D);
}

/* $c5 BITB immediate -**0- */
HD6309_INLINE void bitb_im( void )
{
	UINT8 t,r;
	IMMBYTE(t);
	r = B & t;
	CLR_NZV;
	SET_NZ8(r);
}

/* $1085 BITD immediate -**0- */
HD6309_INLINE void bitd_im( void )
{
	PAIR	t;
	UINT16	r;
	IMMWORD(t);
	r = B & t.w.l;
	CLR_NZV;
	SET_NZ16(r);
}

/* $113c BITMD immediate -**0- */
HD6309_INLINE void bitmd_im( void )
{
	/*
    The following is from Darren A.

    The Z flag is the only condition code that should be affected by BITMD.
    For example, when the "Divide-By-Zero" flag (bit 7) is set, BITMD should
    not set the N flag. It should also NOT clear the V flag (unlike the other
    BIT instructions).

    His comments come from experimentation and differ from Chris Burke
    */

	UINT8 t,r;
	IMMBYTE(t);
	r = MD & t;
	CLR_Z;
	SET_Z8(r);

	MD &= ~(r & 0xc0); /* clear the tested high bits */

}

/* $c6 LDB immediate -**0- */
HD6309_INLINE void ldb_im( void )
{
	IMMBYTE(B);
	CLR_NZV;
	SET_NZ8(B);
}

/* $113d LDMD immediate -**0- */
HD6309_INLINE void ldmd_im( void )
{
	IMMBYTE(MD);
	UpdateState();
}

/* $1186 LDE immediate -**0- */
HD6309_INLINE void lde_im( void )
{
	IMMBYTE(E);
	CLR_NZV;
	SET_NZ8(E);
}

/* $11C6 LDF immediate -**0- */
HD6309_INLINE void ldf_im( void )
{
	IMMBYTE(F);
	CLR_NZV;
	SET_NZ8(F);
}

/* $c8 EORB immediate -**0- */
HD6309_INLINE void eorb_im( void )
{
	UINT8 t;
	IMMBYTE(t);
	B ^= t;
	CLR_NZV;
	SET_NZ8(B);
}

/* $1088 EORD immediate -**0- */
HD6309_INLINE void eord_im( void )
{
	PAIR t;
	IMMWORD(t);
	D ^= t.w.l;
	CLR_NZV;
	SET_NZ16(D);
}

/* $c9 ADCB immediate ***** */
HD6309_INLINE void adcb_im( void )
{
	UINT16 t,r;
	IMMBYTE(t);
	r = B + t + (CC & CC_C);
	CLR_HNZVC;
	SET_FLAGS8(B,t,r);
	SET_H(B,t,r);
	B = r;
}

/* $1089 ADCD immediate ***** */
HD6309_INLINE void adcd_im( void )
{
	PAIR	t;
	UINT32	r;
	IMMWORD(t);
	r = D + t.w.l + (CC & CC_C);
	CLR_NZVC;
	SET_FLAGS16(D,t.w.l,r);
	D = r;
}

/* $cA ORB immediate -**0- */
HD6309_INLINE void orb_im( void )
{
	UINT8 t;
	IMMBYTE(t);
	B |= t;
	CLR_NZV;
	SET_NZ8(B);
}

/* $108a ORD immediate -**0- */
HD6309_INLINE void ord_im( void )
{
	PAIR t;
	IMMWORD(t);
	D |= t.w.l;
	CLR_NZV;
	SET_NZ16(D);
}

/* $cB ADDB immediate ***** */
HD6309_INLINE void addb_im( void )
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
HD6309_INLINE void ldd_im( void )
{
	PAIR	t;

	IMMWORD(t);
	D=t.w.l;
	CLR_NZV;
	SET_NZ16(D);
}

/* $1086 LDW immediate -**0- */
HD6309_INLINE void ldw_im( void )
{
	PAIR	t;
	IMMWORD(t);
	W=t.w.l;
	CLR_NZV;
	SET_NZ16(W);
}

/* $cE LDU (LDS) immediate -**0- */
HD6309_INLINE void ldu_im( void )
{
	IMMWORD(pU);
	CLR_NZV;
	SET_NZ16(U);
}

/* $10cE LDS immediate -**0- */
HD6309_INLINE void lds_im( void )
{
	IMMWORD(pS);
	CLR_NZV;
	SET_NZ16(S);
	hd6309.int_state |= HD6309_LDS;
}

/* $d0 SUBB direct ?**** */
HD6309_INLINE void subb_di( void )
{
	UINT16	  t,r;
	DIRBYTE(t);
	r = B - t;
	CLR_NZVC;
	SET_FLAGS8(B,t,r);
	B = r;
}

/* $1190 SUBE direct ?**** */
HD6309_INLINE void sube_di( void )
{
	UINT16	  t,r;
	DIRBYTE(t);
	r = E - t;
	CLR_NZVC;
	SET_FLAGS8(E,t,r);
	E = r;
}

/* $11d0 SUBF direct ?**** */
HD6309_INLINE void subf_di( void )
{
	UINT16	  t,r;
	DIRBYTE(t);
	r = F - t;
	CLR_NZVC;
	SET_FLAGS8(F,t,r);
	F = r;
}

/* $d1 CMPB direct ?**** */
HD6309_INLINE void cmpb_di( void )
{
	UINT16	  t,r;
	DIRBYTE(t);
	r = B - t;
	CLR_NZVC;
	SET_FLAGS8(B,t,r);
}

/* $1191 CMPE direct ?**** */
HD6309_INLINE void cmpe_di( void )
{
	UINT16	  t,r;
	DIRBYTE(t);
	r = E - t;
	CLR_NZVC;
	SET_FLAGS8(E,t,r);
}

/* $11D1 CMPF direct ?**** */
HD6309_INLINE void cmpf_di( void )
{
	UINT16	  t,r;
	DIRBYTE(t);
	r = F - t;
	CLR_NZVC;
	SET_FLAGS8(F,t,r);
}

/* $d2 SBCB direct ?**** */
HD6309_INLINE void sbcb_di( void )
{
	UINT16	  t,r;
	DIRBYTE(t);
	r = B - t - (CC & CC_C);
	CLR_NZVC;
	SET_FLAGS8(B,t,r);
	B = r;
}

/* $1092 SBCD direct ?**** */
HD6309_INLINE void sbcd_di( void )
{
	PAIR	t;
	UINT32	r;
	DIRWORD(t);
	r = D - t.w.l - (CC & CC_C);
	CLR_NZVC;
	SET_FLAGS16(D,t.w.l,r);
	D = r;
}

/* $d3 ADDD direct -**** */
HD6309_INLINE void addd_di( void )
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

/* $109b ADDW direct -**** */
HD6309_INLINE void addw_di( void )
{
	UINT32 r,d;
	PAIR b;
	DIRWORD(b);
	d = W;
	r = d + b.d;
	CLR_NZVC;
	SET_FLAGS16(d,b.d,r);
	W = r;
}

/* $119b ADDE direct -**** */
HD6309_INLINE void adde_di( void )
{
	UINT16 t,r;
	DIRBYTE(t);
	r = E + t;
	CLR_HNZVC;
	SET_FLAGS8(E,t,r);
	SET_H(E,t,r);
	E = r;
}

/* $11db ADDF direct -**** */
HD6309_INLINE void addf_di( void )
{
	UINT16 t,r;
	DIRBYTE(t);
	r = F + t;
	CLR_HNZVC;
	SET_FLAGS8(F,t,r);
	SET_H(F,t,r);
	F = r;
}

/* $d4 ANDB direct -**0- */
HD6309_INLINE void andb_di( void )
{
	UINT8 t;
	DIRBYTE(t);
	B &= t;
	CLR_NZV;
	SET_NZ8(B);
}

/* $1094 ANDD direct -**0- */
HD6309_INLINE void andd_di( void )
{
	PAIR t;
	DIRWORD(t);
	D &= t.w.l;
	CLR_NZV;
	SET_NZ16(D);
}

/* $d5 BITB direct -**0- */
HD6309_INLINE void bitb_di( void )
{
	UINT8 t,r;
	DIRBYTE(t);
	r = B & t;
	CLR_NZV;
	SET_NZ8(r);
}

/* $1095 BITD direct -**0- */
HD6309_INLINE void bitd_di( void )
{
	PAIR	t;
	UINT16	r;
	DIRWORD(t);
	r = B & t.w.l;
	CLR_NZV;
	SET_NZ16(r);
}

/* $d6 LDB direct -**0- */
HD6309_INLINE void ldb_di( void )
{
	DIRBYTE(B);
	CLR_NZV;
	SET_NZ8(B);
}

/* $1196 LDE direct -**0- */
HD6309_INLINE void lde_di( void )
{
	DIRBYTE(E);
	CLR_NZV;
	SET_NZ8(E);
}

/* $11d6 LDF direct -**0- */
HD6309_INLINE void ldf_di( void )
{
	DIRBYTE(F);
	CLR_NZV;
	SET_NZ8(F);
}

/* $d7 STB direct -**0- */
HD6309_INLINE void stb_di( void )
{
	CLR_NZV;
	SET_NZ8(B);
	DIRECT;
	WM(EAD,B);
}

/* $1197 STE direct -**0- */
HD6309_INLINE void ste_di( void )
{
	CLR_NZV;
	SET_NZ8(E);
	DIRECT;
	WM(EAD,E);
}

/* $11D7 STF direct -**0- */
HD6309_INLINE void stf_di( void )
{
	CLR_NZV;
	SET_NZ8(F);
	DIRECT;
	WM(EAD,F);
}

/* $d8 EORB direct -**0- */
HD6309_INLINE void eorb_di( void )
{
	UINT8 t;
	DIRBYTE(t);
	B ^= t;
	CLR_NZV;
	SET_NZ8(B);
}

/* $1098 EORD direct -**0- */
HD6309_INLINE void eord_di( void )
{
	PAIR t;
	DIRWORD(t);
	D ^= t.w.l;
	CLR_NZV;
	SET_NZ16(D);
}

/* $d9 ADCB direct ***** */
HD6309_INLINE void adcb_di( void )
{
	UINT16 t,r;
	DIRBYTE(t);
	r = B + t + (CC & CC_C);
	CLR_HNZVC;
	SET_FLAGS8(B,t,r);
	SET_H(B,t,r);
	B = r;
}

/* $1099 adcd direct ***** */
HD6309_INLINE void adcd_di( void )
{
	UINT32	r;
	PAIR	t;

	DIRWORD(t);
	r = D + t.w.l + (CC & CC_C);
	CLR_NZVC;
	SET_FLAGS16(D,t.w.l,r);
	D = r;
}

/* $dA ORB direct -**0- */
HD6309_INLINE void orb_di( void )
{
	UINT8 t;
	DIRBYTE(t);
	B |= t;
	CLR_NZV;
	SET_NZ8(B);
}

/* $109a ORD direct -**0- */
HD6309_INLINE void ord_di( void )
{
	PAIR t;
	DIRWORD(t);
	D |= t.w.l;
	CLR_NZV;
	SET_NZ16(D);
}

/* $dB ADDB direct ***** */
HD6309_INLINE void addb_di( void )
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
HD6309_INLINE void ldd_di( void )
{
	PAIR t;
	DIRWORD(t);
	D=t.w.l;
	CLR_NZV;
	SET_NZ16(D);
}

/* $1096 LDW direct -**0- */
HD6309_INLINE void ldw_di( void )
{
	PAIR t;
	DIRWORD(t);
	W=t.w.l;
	CLR_NZV;
	SET_NZ16(W);
}

/* $dD STD direct -**0- */
HD6309_INLINE void std_di( void )
{
	CLR_NZV;
	SET_NZ16(D);
	DIRECT;
	WM16(EAD,&pD);
}

/* $1097 STW direct -**0- */
HD6309_INLINE void stw_di( void )
{
	CLR_NZV;
	SET_NZ16(W);
	DIRECT;
	WM16(EAD,&pW);
}

/* $dE LDU (LDS) direct -**0- */
HD6309_INLINE void ldu_di( void )
{
	DIRWORD(pU);
	CLR_NZV;
	SET_NZ16(U);
}

/* $10dE LDS direct -**0- */
HD6309_INLINE void lds_di( void )
{
	DIRWORD(pS);
	CLR_NZV;
	SET_NZ16(S);
	hd6309.int_state |= HD6309_LDS;
}

/* $dF STU (STS) direct -**0- */
HD6309_INLINE void stu_di( void )
{
	CLR_NZV;
	SET_NZ16(U);
	DIRECT;
	WM16(EAD,&pU);
}

/* $10dF STS direct -**0- */
HD6309_INLINE void sts_di( void )
{
	CLR_NZV;
	SET_NZ16(S);
	DIRECT;
	WM16(EAD,&pS);
}

/* $e0 SUBB indexed ?**** */
HD6309_INLINE void subb_ix( void )
{
	UINT16	  t,r;
	fetch_effective_address();
	t = RM(EAD);
	r = B - t;
	CLR_NZVC;
	SET_FLAGS8(B,t,r);
	B = r;
}

/* $11a0 SUBE indexed ?**** */
HD6309_INLINE void sube_ix( void )
{
	UINT16	  t,r;
	fetch_effective_address();
	t = RM(EAD);
	r = E - t;
	CLR_NZVC;
	SET_FLAGS8(E,t,r);
	E = r;
}

/* $11e0 SUBF indexed ?**** */
HD6309_INLINE void subf_ix( void )
{
	UINT16	  t,r;
	fetch_effective_address();
	t = RM(EAD);
	r = F - t;
	CLR_NZVC;
	SET_FLAGS8(F,t,r);
	F = r;
}

/* $e1 CMPB indexed ?**** */
HD6309_INLINE void cmpb_ix( void )
{
	UINT16	  t,r;
	fetch_effective_address();
	t = RM(EAD);
	r = B - t;
	CLR_NZVC;
	SET_FLAGS8(B,t,r);
}

/* $11a1 CMPE indexed ?**** */
HD6309_INLINE void cmpe_ix( void )
{
	UINT16	  t,r;
	fetch_effective_address();
	t = RM(EAD);
	r = E - t;
	CLR_NZVC;
	SET_FLAGS8(E,t,r);
}

/* $11e1 CMPF indexed ?**** */
HD6309_INLINE void cmpf_ix( void )
{
	UINT16	  t,r;
	fetch_effective_address();
	t = RM(EAD);
	r = F - t;
	CLR_NZVC;
	SET_FLAGS8(F,t,r);
}

/* $e2 SBCB indexed ?**** */
HD6309_INLINE void sbcb_ix( void )
{
	UINT16	  t,r;
	fetch_effective_address();
	t = RM(EAD);
	r = B - t - (CC & CC_C);
	CLR_NZVC;
	SET_FLAGS8(B,t,r);
	B = r;
}

/* $10a2 SBCD indexed ?**** */
HD6309_INLINE void sbcd_ix( void )
{
	UINT32	  t,r;
	fetch_effective_address();
	t = RM16(EAD);
	r = D - t - (CC & CC_C);
	CLR_NZVC;
	SET_FLAGS16(D,t,r);
	D = r;
}

/* $e3 ADDD indexed -**** */
HD6309_INLINE void addd_ix( void )
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

/* $10ab ADDW indexed -**** */
HD6309_INLINE void addw_ix( void )
{
	UINT32 r,d;
	PAIR b;
	fetch_effective_address();
	b.d=RM16(EAD);
	d = W;
	r = d + b.d;
	CLR_NZVC;
	SET_FLAGS16(d,b.d,r);
	W = r;
}

/* $11ab ADDE indexed -**** */
HD6309_INLINE void adde_ix( void )
{
	UINT16 t,r;
	fetch_effective_address();
	t = RM(EAD);
	r = E + t;
	CLR_HNZVC;
	SET_FLAGS8(E,t,r);
	SET_H(E,t,r);
	E = r;
}

/* $11eb ADDF indexed -**** */
HD6309_INLINE void addf_ix( void )
{
	UINT16 t,r;
	fetch_effective_address();
	t = RM(EAD);
	r = F + t;
	CLR_HNZVC;
	SET_FLAGS8(F,t,r);
	SET_H(F,t,r);
	F = r;
}

/* $e4 ANDB indexed -**0- */
HD6309_INLINE void andb_ix( void )
{
	fetch_effective_address();
	B &= RM(EAD);
	CLR_NZV;
	SET_NZ8(B);
}

/* $10a4 ANDD indexed -**0- */
HD6309_INLINE void andd_ix( void )
{
	fetch_effective_address();
	D &= RM16(EAD);
	CLR_NZV;
	SET_NZ16(D);
}

/* $e5 BITB indexed -**0- */
HD6309_INLINE void bitb_ix( void )
{
	UINT8 r;
	fetch_effective_address();
	r = B & RM(EAD);
	CLR_NZV;
	SET_NZ8(r);
}

/* $10a5 BITD indexed -**0- */
HD6309_INLINE void bitd_ix( void )
{
	UINT16 r;
	fetch_effective_address();
	r = D & RM16(EAD);
	CLR_NZV;
	SET_NZ16(r);
}

/* $e6 LDB indexed -**0- */
HD6309_INLINE void ldb_ix( void )
{
	fetch_effective_address();
	B = RM(EAD);
	CLR_NZV;
	SET_NZ8(B);
}

/* $11a6 LDE indexed -**0- */
HD6309_INLINE void lde_ix( void )
{
	fetch_effective_address();
	E = RM(EAD);
	CLR_NZV;
	SET_NZ8(E);
}

/* $11e6 LDF indexed -**0- */
HD6309_INLINE void ldf_ix( void )
{
	fetch_effective_address();
	F = RM(EAD);
	CLR_NZV;
	SET_NZ8(F);
}

/* $e7 STB indexed -**0- */
HD6309_INLINE void stb_ix( void )
{
	fetch_effective_address();
	CLR_NZV;
	SET_NZ8(B);
	WM(EAD,B);
}

/* $11a7 STE indexed -**0- */
HD6309_INLINE void ste_ix( void )
{
	fetch_effective_address();
	CLR_NZV;
	SET_NZ8(E);
	WM(EAD,E);
}

/* $11e7 STF indexed -**0- */
HD6309_INLINE void stf_ix( void )
{
	fetch_effective_address();
	CLR_NZV;
	SET_NZ8(F);
	WM(EAD,F);
}

/* $e8 EORB indexed -**0- */
HD6309_INLINE void eorb_ix( void )
{
	fetch_effective_address();
	B ^= RM(EAD);
	CLR_NZV;
	SET_NZ8(B);
}

/* $10a8 EORD indexed -**0- */
HD6309_INLINE void eord_ix( void )
{
	fetch_effective_address();
	D ^= RM16(EAD);
	CLR_NZV;
	SET_NZ16(D);
}

/* $e9 ADCB indexed ***** */
HD6309_INLINE void adcb_ix( void )
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

/* $10a9 ADCD indexed ***** */
HD6309_INLINE void adcd_ix( void )
{
	UINT32	r;
	PAIR	t;
	fetch_effective_address();
	t.d = RM16(EAD);
	r = D + t.d + (CC & CC_C);
	CLR_NZVC;
	SET_FLAGS16(D,t.d,r);
	D = r;
}

/* $eA ORB indexed -**0- */
HD6309_INLINE void orb_ix( void )
{
	fetch_effective_address();
	B |= RM(EAD);
	CLR_NZV;
	SET_NZ8(B);
}

/* $10aa ORD indexed -**0- */
HD6309_INLINE void ord_ix( void )
{
	fetch_effective_address();
	D |= RM16(EAD);
	CLR_NZV;
	SET_NZ16(D);
}

/* $eB ADDB indexed ***** */
HD6309_INLINE void addb_ix( void )
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
HD6309_INLINE void ldd_ix( void )
{
	fetch_effective_address();
	D=RM16(EAD);
	CLR_NZV; SET_NZ16(D);
}

/* $10a6 LDW indexed -**0- */
HD6309_INLINE void ldw_ix( void )
{
	fetch_effective_address();
	W=RM16(EAD);
	CLR_NZV; SET_NZ16(W);
}

/* $eD STD indexed -**0- */
HD6309_INLINE void std_ix( void )
{
	fetch_effective_address();
	CLR_NZV;
	SET_NZ16(D);
	WM16(EAD,&pD);
}

/* $10a7 STW indexed -**0- */
HD6309_INLINE void stw_ix( void )
{
	fetch_effective_address();
	CLR_NZV;
	SET_NZ16(W);
	WM16(EAD,&pW);
}

/* $eE LDU (LDS) indexed -**0- */
HD6309_INLINE void ldu_ix( void )
{
	fetch_effective_address();
	U=RM16(EAD);
	CLR_NZV;
	SET_NZ16(U);
}

/* $10eE LDS indexed -**0- */
HD6309_INLINE void lds_ix( void )
{
	fetch_effective_address();
	S=RM16(EAD);
	CLR_NZV;
	SET_NZ16(S);
	hd6309.int_state |= HD6309_LDS;
}

/* $eF STU (STS) indexed -**0- */
HD6309_INLINE void stu_ix( void )
{
	fetch_effective_address();
	CLR_NZV;
	SET_NZ16(U);
	WM16(EAD,&pU);
}

/* $10eF STS indexed -**0- */
HD6309_INLINE void sts_ix( void )
{
	fetch_effective_address();
	CLR_NZV;
	SET_NZ16(S);
	WM16(EAD,&pS);
}

/* $f0 SUBB extended ?**** */
HD6309_INLINE void subb_ex( void )
{
	UINT16	  t,r;
	EXTBYTE(t);
	r = B - t;
	CLR_NZVC;
	SET_FLAGS8(B,t,r);
	B = r;
}

/* $11b0 SUBE extended ?**** */
HD6309_INLINE void sube_ex( void )
{
	UINT16	  t,r;
	EXTBYTE(t);
	r = E - t;
	CLR_NZVC;
	SET_FLAGS8(E,t,r);
	E = r;
}

/* $11f0 SUBF extended ?**** */
HD6309_INLINE void subf_ex( void )
{
	UINT16	  t,r;
	EXTBYTE(t);
	r = F - t;
	CLR_NZVC;
	SET_FLAGS8(F,t,r);
	F = r;
}

/* $f1 CMPB extended ?**** */
HD6309_INLINE void cmpb_ex( void )
{
	UINT16	  t,r;
	EXTBYTE(t);
	r = B - t;
	CLR_NZVC;
	SET_FLAGS8(B,t,r);
}

/* $11b1 CMPE extended ?**** */
HD6309_INLINE void cmpe_ex( void )
{
	UINT16	  t,r;
	EXTBYTE(t);
	r = E - t;
	CLR_NZVC;
	SET_FLAGS8(E,t,r);
}

/* $11f1 CMPF extended ?**** */
HD6309_INLINE void cmpf_ex( void )
{
	UINT16	  t,r;
	EXTBYTE(t);
	r = F - t;
	CLR_NZVC;
	SET_FLAGS8(F,t,r);
}

/* $f2 SBCB extended ?**** */
HD6309_INLINE void sbcb_ex( void )
{
	UINT16	  t,r;
	EXTBYTE(t);
	r = B - t - (CC & CC_C);
	CLR_NZVC;
	SET_FLAGS8(B,t,r);
	B = r;
}

/* $10b2 SBCD extended ?**** */
HD6309_INLINE void sbcd_ex( void )
{
	PAIR t = {{0,0,0,0}};
	UINT32 r;

	EXTWORD(t);
	r = D - t.w.l - (CC & CC_C);
	CLR_NZVC;
	SET_FLAGS16(D,t.w.l,r);
	D = r;
}

/* $f3 ADDD extended -**** */
HD6309_INLINE void addd_ex( void )
{
	UINT32 r,d;
	PAIR b = {{0,0,0,0}};
	EXTWORD(b);
	d = D;
	r = d + b.d;
	CLR_NZVC;
	SET_FLAGS16(d,b.d,r);
	D = r;
}

/* $10bb ADDW extended -**** */
HD6309_INLINE void addw_ex( void )
{
	UINT32 r,d;
	PAIR b = {{0,0,0,0}};
	EXTWORD(b);
	d = W;
	r = d + b.d;
	CLR_NZVC;
	SET_FLAGS16(d,b.d,r);
	W = r;
}

/* $11bb ADDE extended -**** */
HD6309_INLINE void adde_ex( void )
{
	UINT16 t,r;
	EXTBYTE(t);
	r = E + t;
	CLR_HNZVC;
	SET_FLAGS8(E,t,r);
	SET_H(E,t,r);
	E = r;
}

/* $11fb ADDF extended -**** */
HD6309_INLINE void addf_ex( void )
{
	UINT16 t,r;
	EXTBYTE(t);
	r = F + t;
	CLR_HNZVC;
	SET_FLAGS8(F,t,r);
	SET_H(F,t,r);
	F = r;
}

/* $f4 ANDB extended -**0- */
HD6309_INLINE void andb_ex( void )
{
	UINT8 t;
	EXTBYTE(t);
	B &= t;
	CLR_NZV;
	SET_NZ8(B);
}

/* $10b4 ANDD extended -**0- */
HD6309_INLINE void andd_ex( void )
{
	PAIR t = {{0,0,0,0}};
	EXTWORD(t);
	D &= t.w.l;
	CLR_NZV;
	SET_NZ16(D);
}

/* $f5 BITB extended -**0- */
HD6309_INLINE void bitb_ex( void )
{
	UINT8 t,r;
	EXTBYTE(t);
	r = B & t;
	CLR_NZV;
	SET_NZ8(r);
}

/* $10b5 BITD extended -**0- */
HD6309_INLINE void bitd_ex( void )
{
	PAIR t = {{0,0,0,0}};
	UINT8 r;
	EXTWORD(t);
	r = B & t.w.l;
	CLR_NZV;
	SET_NZ16(r);
}

/* $f6 LDB extended -**0- */
HD6309_INLINE void ldb_ex( void )
{
	EXTBYTE(B);
	CLR_NZV;
	SET_NZ8(B);
}

/* $11b6 LDE extended -**0- */
HD6309_INLINE void lde_ex( void )
{
	EXTBYTE(E);
	CLR_NZV;
	SET_NZ8(E);
}

/* $11f6 LDF extended -**0- */
HD6309_INLINE void ldf_ex( void )
{
	EXTBYTE(F);
	CLR_NZV;
	SET_NZ8(F);
}

/* $f7 STB extended -**0- */
HD6309_INLINE void stb_ex( void )
{
	CLR_NZV;
	SET_NZ8(B);
	EXTENDED;
	WM(EAD,B);
}

/* $11b7 STE extended -**0- */
HD6309_INLINE void ste_ex( void )
{
	CLR_NZV;
	SET_NZ8(E);
	EXTENDED;
	WM(EAD,E);
}

/* $11f7 STF extended -**0- */
HD6309_INLINE void stf_ex( void )
{
	CLR_NZV;
	SET_NZ8(F);
	EXTENDED;
	WM(EAD,F);
}

/* $f8 EORB extended -**0- */
HD6309_INLINE void eorb_ex( void )
{
	UINT8 t;
	EXTBYTE(t);
	B ^= t;
	CLR_NZV;
	SET_NZ8(B);
}

/* $10b8 EORD extended -**0- */
HD6309_INLINE void eord_ex( void )
{
	PAIR t = {{0,0,0,0}};
	EXTWORD(t);
	D ^= t.w.l;
	CLR_NZV;
	SET_NZ16(D);
}

/* $f9 ADCB extended ***** */
HD6309_INLINE void adcb_ex( void )
{
	UINT16 t,r;
	EXTBYTE(t);
	r = B + t + (CC & CC_C);
	CLR_HNZVC;
	SET_FLAGS8(B,t,r);
	SET_H(B,t,r);
	B = r;
}

/* $10b9 ADCD extended ***** */
HD6309_INLINE void adcd_ex( void )
{
	UINT32	r;
	PAIR	t;
	EXTWORD(t);
	r = D + t.w.l + (CC & CC_C);
	CLR_NZVC;
	SET_FLAGS16(D,t.w.l,r);
	D = r;
}

/* $fA ORB extended -**0- */
HD6309_INLINE void orb_ex( void )
{
	UINT8 t;
	EXTBYTE(t);
	B |= t;
	CLR_NZV;
	SET_NZ8(B);
}

/* $10ba ORD extended -**0- */
HD6309_INLINE void ord_ex( void )
{
	PAIR t = {{0,0,0,0}};
	EXTWORD(t);
	D |= t.w.l;
	CLR_NZV;
	SET_NZ8(D);
}

/* $fB ADDB extended ***** */
HD6309_INLINE void addb_ex( void )
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
HD6309_INLINE void ldd_ex( void )
{
	EXTWORD(pD);
	CLR_NZV;
	SET_NZ16(D);
}

/* $10b6 LDW extended -**0- */
HD6309_INLINE void ldw_ex( void )
{
	EXTWORD(pW);
	CLR_NZV;
	SET_NZ16(W);
}

/* $fD STD extended -**0- */
HD6309_INLINE void std_ex( void )
{
	CLR_NZV;
	SET_NZ16(D);
	EXTENDED;
	WM16(EAD,&pD);
}

/* $10b7 STW extended -**0- */
HD6309_INLINE void stw_ex( void )
{
	CLR_NZV;
	SET_NZ16(W);
	EXTENDED;
	WM16(EAD,&pW);
}

/* $fE LDU (LDS) extended -**0- */
HD6309_INLINE void ldu_ex( void )
{
	EXTWORD(pU);
	CLR_NZV;
	SET_NZ16(U);
}

/* $10fE LDS extended -**0- */
HD6309_INLINE void lds_ex( void )
{
	EXTWORD(pS);
	CLR_NZV;
	SET_NZ16(S);
	hd6309.int_state |= HD6309_LDS;
}

/* $fF STU (STS) extended -**0- */
HD6309_INLINE void stu_ex( void )
{
	CLR_NZV;
	SET_NZ16(U);
	EXTENDED;
	WM16(EAD,&pU);
}

/* $10fF STS extended -**0- */
HD6309_INLINE void sts_ex( void )
{
	CLR_NZV;
	SET_NZ16(S);
	EXTENDED;
	WM16(EAD,&pS);
}

/* $10xx opcodes */
HD6309_INLINE void pref10( void )
{
	UINT8 ireg2 = ROP(PCD);
	PC++;

#ifdef BIG_SWITCH
	switch( ireg2 )
	{
		case 0x21: lbrn();			break;
		case 0x22: lbhi();			break;
		case 0x23: lbls();			break;
		case 0x24: lbcc();			break;
		case 0x25: lbcs();			break;
		case 0x26: lbne();			break;
		case 0x27: lbeq();			break;
		case 0x28: lbvc();			break;
		case 0x29: lbvs();			break;
		case 0x2a: lbpl();			break;
		case 0x2b: lbmi();			break;
		case 0x2c: lbge();			break;
		case 0x2d: lblt();			break;
		case 0x2e: lbgt();			break;
		case 0x2f: lble();			break;

		case 0x30: addr_r();		break;
		case 0x31: adcr();			break;
		case 0x32: subr();			break;
		case 0x33: sbcr();			break;
		case 0x34: andr();			break;
		case 0x35: orr();			break;
		case 0x36: eorr();			break;
		case 0x37: cmpr();			break;
		case 0x38: pshsw(); 		break;
		case 0x39: pulsw(); 		break;
		case 0x3a: pshuw(); 		break;
		case 0x3b: puluw(); 		break;
		case 0x3f: swi2();		    break;

		case 0x40: negd();			break;
		case 0x43: comd();			break;
		case 0x44: lsrd();			break;
		case 0x46: rord();			break;
		case 0x47: asrd();			break;
		case 0x48: asld();			break;
		case 0x49: rold();			break;
		case 0x4a: decd();			break;
		case 0x4c: incd();			break;
		case 0x4d: tstd();			break;
		case 0x4f: clrd();			break;

		case 0x53: comw();			break;
		case 0x54: lsrw();			break;
		case 0x56: rorw();			break;
		case 0x59: rolw();			break;
		case 0x5a: decw();			break;
		case 0x5c: incw();			break;
		case 0x5d: tstw();			break;
		case 0x5f: clrw();			break;

		case 0x80: subw_im();		break;
		case 0x81: cmpw_im();		break;
		case 0x82: sbcd_im();		break;
		case 0x83: cmpd_im();		break;
		case 0x84: andd_im();		break;
		case 0x85: bitd_im();		break;
		case 0x86: ldw_im();		break;
		case 0x88: eord_im();		break;
		case 0x89: adcd_im();		break;
		case 0x8a: ord_im();		break;
		case 0x8b: addw_im();		break;
		case 0x8c: cmpy_im();		break;
		case 0x8e: ldy_im();		break;

		case 0x90: subw_di();		break;
		case 0x91: cmpw_di();		break;
		case 0x92: sbcd_di();		break;
		case 0x93: cmpd_di();		break;
		case 0x94: andd_di();		break;
		case 0x95: bitd_di();		break;
		case 0x96: ldw_di();		break;
		case 0x97: stw_di();		break;
		case 0x98: eord_di();		break;
		case 0x99: adcd_di();		break;
		case 0x9a: ord_di();		break;
		case 0x9b: addw_di();		break;
		case 0x9c: cmpy_di();		break;
		case 0x9e: ldy_di();		break;
		case 0x9f: sty_di();		break;

		case 0xa0: subw_ix();		break;
		case 0xa1: cmpw_ix();		break;
		case 0xa2: sbcd_ix();		break;
		case 0xa3: cmpd_ix();		break;
		case 0xa4: andd_ix();		break;
		case 0xa5: bitd_ix();		break;
		case 0xa6: ldw_ix();		break;
		case 0xa7: stw_ix();		break;
		case 0xa8: eord_ix();		break;
		case 0xa9: adcd_ix();		break;
		case 0xaa: ord_ix();		break;
		case 0xab: addw_ix();		break;
		case 0xac: cmpy_ix();		break;
		case 0xae: ldy_ix();		break;
		case 0xaf: sty_ix();		break;

		case 0xb0: subw_ex();		break;
		case 0xb1: cmpw_ex();		break;
		case 0xb2: sbcd_ex();		break;
		case 0xb3: cmpd_ex();		break;
		case 0xb4: andd_ex();		break;
		case 0xb5: bitd_ex();		break;
		case 0xb6: ldw_ex();		break;
		case 0xb7: stw_ex();		break;
		case 0xb8: eord_ex();		break;
		case 0xb9: adcd_ex();		break;
		case 0xba: ord_ex();		break;
		case 0xbb: addw_ex();		break;
		case 0xbc: cmpy_ex();		break;
		case 0xbe: ldy_ex();		break;
		case 0xbf: sty_ex();		break;

		case 0xce: lds_im();		break;

		case 0xdc: ldq_di();		break;
		case 0xdd: stq_di();		break;
		case 0xde: lds_di();		break;
		case 0xdf: sts_di();		break;

		case 0xec: ldq_ix();		break;
		case 0xed: stq_ix();		break;
		case 0xee: lds_ix();		break;
		case 0xef: sts_ix();		break;

		case 0xfc: ldq_ex();		break;
		case 0xfd: stq_ex();		break;
		case 0xfe: lds_ex();		break;
		case 0xff: sts_ex();		break;

		default:  IIError();        break;
	}
#else

	(*hd6309_page01[ireg2])();

#endif /* BIG_SWITCH */

	hd6309_ICount -= cycle_counts_page01[ireg2];
}

/* $11xx opcodes */
HD6309_INLINE void pref11( void )
{
	UINT8 ireg2 = ROP(PCD);
	PC++;

#ifdef BIG_SWITCH
	switch( ireg2 )
	{
		case 0x30: band();			break;
		case 0x31: biand(); 		break;
		case 0x32: bor();			break;
		case 0x33: bior();			break;
		case 0x34: beor();			break;
		case 0x35: bieor(); 		break;
		case 0x36: ldbt();			break;
		case 0x37: stbt();			break;
		case 0x38: tfmpp(); 		break;	/* Timing for TFM is actually 6+3n.        */
		case 0x39: tfmmm(); 		break;	/* To avoid saving the state, I decided    */
		case 0x3a: tfmpc(); 		break;	/* to push the initial 6 cycles to the end */
		case 0x3b: tfmcp(); 		break;  /* We will soon see how this fairs!        */
		case 0x3c: bitmd_im();		break;
		case 0x3d: ldmd_im();		break;
		case 0x3f: swi3();			break;

		case 0x43: come();			break;
		case 0x4a: dece();			break;
		case 0x4c: ince();			break;
		case 0x4d: tste();			break;
		case 0x4f: clre();			break;

		case 0x53: comf();			break;
		case 0x5a: decf();			break;
		case 0x5c: incf();			break;
		case 0x5d: tstf();			break;
		case 0x5f: clrf();			break;

		case 0x80: sube_im();		break;
		case 0x81: cmpe_im();		break;
		case 0x83: cmpu_im();		break;
		case 0x86: lde_im();		break;
		case 0x8b: adde_im();		break;
		case 0x8c: cmps_im();		break;
		case 0x8d: divd_im();		break;
		case 0x8e: divq_im();		break;
		case 0x8f: muld_im();		break;

		case 0x90: sube_di();		break;
		case 0x91: cmpe_di();		break;
		case 0x93: cmpu_di();		break;
		case 0x96: lde_di();		break;
		case 0x97: ste_di();		break;
		case 0x9b: adde_di();		break;
		case 0x9c: cmps_di();		break;
		case 0x9d: divd_di();		break;
		case 0x9e: divq_di();		break;
		case 0x9f: muld_di();		break;

		case 0xa0: sube_ix();		break;
		case 0xa1: cmpe_ix();		break;
		case 0xa3: cmpu_ix();		break;
		case 0xa6: lde_ix();		break;
		case 0xa7: ste_ix();		break;
		case 0xab: adde_ix();		break;
		case 0xac: cmps_ix();		break;
		case 0xad: divd_ix();		break;
		case 0xae: divq_ix();		break;
		case 0xaf: muld_ix();		break;

		case 0xb0: sube_ex();		break;
		case 0xb1: cmpe_ex();		break;
		case 0xb3: cmpu_ex();		break;
		case 0xb6: lde_ex();		break;
		case 0xb7: ste_ex();		break;
		case 0xbb: adde_ex();		break;
		case 0xbc: cmps_ex();		break;
		case 0xbd: divd_ex();		break;
		case 0xbe: divq_ex();		break;
		case 0xbf: muld_ex();		break;

		case 0xc0: subf_im();		break;
		case 0xc1: cmpf_im();		break;
		case 0xc6: ldf_im();		break;
		case 0xcb: addf_im();		break;

		case 0xd0: subf_di();		break;
		case 0xd1: cmpf_di();		break;
		case 0xd6: ldf_di();		break;
		case 0xd7: stf_di();		break;
		case 0xdb: addf_di();		break;

		case 0xe0: subf_ix();		break;
		case 0xe1: cmpf_ix();		break;
		case 0xe6: ldf_ix();		break;
		case 0xe7: stf_ix();		break;
		case 0xeb: addf_ix();		break;

		case 0xf0: subf_ex();		break;
		case 0xf1: cmpf_ex();		break;
		case 0xf6: ldf_ex();		break;
		case 0xf7: stf_ex();		break;
		case 0xfb: addf_ex();		break;

		default:   IIError();		break;
	}
#else

	(*hd6309_page11[ireg2])();

#endif /* BIG_SWITCH */
	hd6309_ICount -= cycle_counts_page11[ireg2];
}

#ifdef __cplusplus
}
#endif

/* execute instructions on this CPU until icount expires */
int hd6309_execute(int cycles)	/* NS 970908 */
{
	hd6309_ICount = cycles - hd6309.extra_cycles;
	hd6309.extra_cycles = 0;

	if (hd6309.int_state & (HD6309_CWAI | HD6309_SYNC))
	{
//		debugger_instruction_hook(Machine, PCD);
		hd6309_ICount = 0;
	}
	else
	{
		do
		{
			pPPC = pPC;

//			debugger_instruction_hook(Machine, PCD);

			hd6309.ireg = ROP(PCD);
			PC++;

#ifdef BIG_SWITCH
			switch( hd6309.ireg )
			{
			case 0x00: neg_di();   				break;
			case 0x01: oim_di();   				break;
			case 0x02: aim_di();   				break;
			case 0x03: com_di();   				break;
			case 0x04: lsr_di();   				break;
			case 0x05: eim_di();   				break;
			case 0x06: ror_di();   				break;
			case 0x07: asr_di();   				break;
			case 0x08: asl_di();   				break;
			case 0x09: rol_di();   				break;
			case 0x0a: dec_di();   				break;
			case 0x0b: tim_di();   				break;
			case 0x0c: inc_di();   				break;
			case 0x0d: tst_di();   				break;
			case 0x0e: jmp_di();   				break;
			case 0x0f: clr_di();   				break;
			case 0x10: pref10();				break;
			case 0x11: pref11();				break;
			case 0x12: nop();	   				break;
			case 0x13: sync();	   				break;
			case 0x14: sexw();	   				break;
			case 0x15: IIError();				break;
			case 0x16: lbra();	   				break;
			case 0x17: lbsr();	   				break;
			case 0x18: IIError();				break;
			case 0x19: daa();	   				break;
			case 0x1a: orcc();	   				break;
			case 0x1b: IIError();				break;
			case 0x1c: andcc();    				break;
			case 0x1d: sex();	   				break;
			case 0x1e: exg();	   				break;
			case 0x1f: tfr();	   				break;
			case 0x20: bra();	   				break;
			case 0x21: brn();	   				break;
			case 0x22: bhi();	   				break;
			case 0x23: bls();	   				break;
			case 0x24: bcc();	   				break;
			case 0x25: bcs();	   				break;
			case 0x26: bne();	   				break;
			case 0x27: beq();	   				break;
			case 0x28: bvc();	   				break;
			case 0x29: bvs();	   				break;
			case 0x2a: bpl();	   				break;
			case 0x2b: bmi();	   				break;
			case 0x2c: bge();	   				break;
			case 0x2d: blt();	   				break;
			case 0x2e: bgt();	   				break;
			case 0x2f: ble();	   				break;
			case 0x30: leax();	   				break;
			case 0x31: leay();	   				break;
			case 0x32: leas();	   				break;
			case 0x33: leau();	   				break;
			case 0x34: pshs();	   				break;
			case 0x35: puls();	   				break;
			case 0x36: pshu();	   				break;
			case 0x37: pulu();	   				break;
			case 0x38: IIError();				break;
			case 0x39: rts();	   				break;
			case 0x3a: abx();	   				break;
			case 0x3b: rti();	   				break;
			case 0x3c: cwai();					break;
			case 0x3d: mul();					break;
			case 0x3e: IIError();				break;
			case 0x3f: swi();					break;
			case 0x40: nega();	   				break;
			case 0x41: IIError();				break;
			case 0x42: IIError();				break;
			case 0x43: coma();	   				break;
			case 0x44: lsra();	   				break;
			case 0x45: IIError();				break;
			case 0x46: rora();	   				break;
			case 0x47: asra();	   				break;
			case 0x48: asla();	   				break;
			case 0x49: rola();	   				break;
			case 0x4a: deca();	   				break;
			case 0x4b: IIError();				break;
			case 0x4c: inca();	   				break;
			case 0x4d: tsta();	   				break;
			case 0x4e: IIError();				break;
			case 0x4f: clra();	   				break;
			case 0x50: negb();	   				break;
			case 0x51: IIError();				break;
			case 0x52: IIError();				break;
			case 0x53: comb();	   				break;
			case 0x54: lsrb();	   				break;
			case 0x55: IIError();				break;
			case 0x56: rorb();	   				break;
			case 0x57: asrb();	   				break;
			case 0x58: aslb();	   				break;
			case 0x59: rolb();	   				break;
			case 0x5a: decb();	   				break;
			case 0x5b: IIError();				break;
			case 0x5c: incb();	   				break;
			case 0x5d: tstb();	   				break;
			case 0x5e: IIError();				break;
			case 0x5f: clrb();	   				break;
			case 0x60: neg_ix();   				break;
			case 0x61: oim_ix();   				break;
			case 0x62: aim_ix();   				break;
			case 0x63: com_ix();   				break;
			case 0x64: lsr_ix();   				break;
			case 0x65: eim_ix();   				break;
			case 0x66: ror_ix();   				break;
			case 0x67: asr_ix();   				break;
			case 0x68: asl_ix();   				break;
			case 0x69: rol_ix();   				break;
			case 0x6a: dec_ix();   				break;
			case 0x6b: tim_ix();   				break;
			case 0x6c: inc_ix();   				break;
			case 0x6d: tst_ix();   				break;
			case 0x6e: jmp_ix();   				break;
			case 0x6f: clr_ix();   				break;
			case 0x70: neg_ex();   				break;
			case 0x71: oim_ex();   				break;
			case 0x72: aim_ex();   				break;
			case 0x73: com_ex();   				break;
			case 0x74: lsr_ex();   				break;
			case 0x75: eim_ex();   				break;
			case 0x76: ror_ex();   				break;
			case 0x77: asr_ex();   				break;
			case 0x78: asl_ex();   				break;
			case 0x79: rol_ex();   				break;
			case 0x7a: dec_ex();   				break;
			case 0x7b: tim_ex();   				break;
			case 0x7c: inc_ex();   				break;
			case 0x7d: tst_ex();   				break;
			case 0x7e: jmp_ex();   				break;
			case 0x7f: clr_ex();   				break;
			case 0x80: suba_im();  				break;
			case 0x81: cmpa_im();  				break;
			case 0x82: sbca_im();  				break;
			case 0x83: subd_im();  				break;
			case 0x84: anda_im();  				break;
			case 0x85: bita_im();  				break;
			case 0x86: lda_im();   				break;
			case 0x87: IIError(); 				break;
			case 0x88: eora_im();  				break;
			case 0x89: adca_im();  				break;
			case 0x8a: ora_im();   				break;
			case 0x8b: adda_im();  				break;
			case 0x8c: cmpx_im();  				break;
			case 0x8d: bsr();	   				break;
			case 0x8e: ldx_im();   				break;
			case 0x8f: IIError();  				break;
			case 0x90: suba_di();  				break;
			case 0x91: cmpa_di();  				break;
			case 0x92: sbca_di();  				break;
			case 0x93: subd_di();  				break;
			case 0x94: anda_di();  				break;
			case 0x95: bita_di();  				break;
			case 0x96: lda_di();   				break;
			case 0x97: sta_di();   				break;
			case 0x98: eora_di();  				break;
			case 0x99: adca_di();  				break;
			case 0x9a: ora_di();   				break;
			case 0x9b: adda_di();  				break;
			case 0x9c: cmpx_di();  				break;
			case 0x9d: jsr_di();   				break;
			case 0x9e: ldx_di();   				break;
			case 0x9f: stx_di();   				break;
			case 0xa0: suba_ix();  				break;
			case 0xa1: cmpa_ix();  				break;
			case 0xa2: sbca_ix();  				break;
			case 0xa3: subd_ix();  				break;
			case 0xa4: anda_ix();  				break;
			case 0xa5: bita_ix();  				break;
			case 0xa6: lda_ix();   				break;
			case 0xa7: sta_ix();   				break;
			case 0xa8: eora_ix();  				break;
			case 0xa9: adca_ix();  				break;
			case 0xaa: ora_ix();   				break;
			case 0xab: adda_ix();  				break;
			case 0xac: cmpx_ix();  				break;
			case 0xad: jsr_ix();   				break;
			case 0xae: ldx_ix();   				break;
			case 0xaf: stx_ix();   				break;
			case 0xb0: suba_ex();  				break;
			case 0xb1: cmpa_ex();  				break;
			case 0xb2: sbca_ex();  				break;
			case 0xb3: subd_ex();  				break;
			case 0xb4: anda_ex();  				break;
			case 0xb5: bita_ex();  				break;
			case 0xb6: lda_ex();   				break;
			case 0xb7: sta_ex();   				break;
			case 0xb8: eora_ex();  				break;
			case 0xb9: adca_ex();  				break;
			case 0xba: ora_ex();   				break;
			case 0xbb: adda_ex();  				break;
			case 0xbc: cmpx_ex();  				break;
			case 0xbd: jsr_ex();   				break;
			case 0xbe: ldx_ex();   				break;
			case 0xbf: stx_ex();   				break;
			case 0xc0: subb_im();  				break;
			case 0xc1: cmpb_im();  				break;
			case 0xc2: sbcb_im();  				break;
			case 0xc3: addd_im();  				break;
			case 0xc4: andb_im();  				break;
			case 0xc5: bitb_im();  				break;
			case 0xc6: ldb_im();   				break;
			case 0xc7: IIError(); 				break;
			case 0xc8: eorb_im();  				break;
			case 0xc9: adcb_im();  				break;
			case 0xca: orb_im();   				break;
			case 0xcb: addb_im();  				break;
			case 0xcc: ldd_im();   				break;
			case 0xcd: ldq_im();   				break; /* in m6809 was std_im */
			case 0xce: ldu_im();   				break;
			case 0xcf: IIError();  				break;
			case 0xd0: subb_di();  				break;
			case 0xd1: cmpb_di();  				break;
			case 0xd2: sbcb_di();  				break;
			case 0xd3: addd_di();  				break;
			case 0xd4: andb_di();  				break;
			case 0xd5: bitb_di();  				break;
			case 0xd6: ldb_di();   				break;
			case 0xd7: stb_di();   				break;
			case 0xd8: eorb_di();  				break;
			case 0xd9: adcb_di();  				break;
			case 0xda: orb_di();   				break;
			case 0xdb: addb_di();  				break;
			case 0xdc: ldd_di();   				break;
			case 0xdd: std_di();   				break;
			case 0xde: ldu_di();   				break;
			case 0xdf: stu_di();   				break;
			case 0xe0: subb_ix();  				break;
			case 0xe1: cmpb_ix();  				break;
			case 0xe2: sbcb_ix();  				break;
			case 0xe3: addd_ix();  				break;
			case 0xe4: andb_ix();  				break;
			case 0xe5: bitb_ix();  				break;
			case 0xe6: ldb_ix();   				break;
			case 0xe7: stb_ix();   				break;
			case 0xe8: eorb_ix();  				break;
			case 0xe9: adcb_ix();  				break;
			case 0xea: orb_ix();   				break;
			case 0xeb: addb_ix();  				break;
			case 0xec: ldd_ix();   				break;
			case 0xed: std_ix();   				break;
			case 0xee: ldu_ix();   				break;
			case 0xef: stu_ix();   				break;
			case 0xf0: subb_ex();  				break;
			case 0xf1: cmpb_ex();  				break;
			case 0xf2: sbcb_ex();  				break;
			case 0xf3: addd_ex();  				break;
			case 0xf4: andb_ex();  				break;
			case 0xf5: bitb_ex();  				break;
			case 0xf6: ldb_ex();   				break;
			case 0xf7: stb_ex();   				break;
			case 0xf8: eorb_ex();  				break;
			case 0xf9: adcb_ex();  				break;
			case 0xfa: orb_ex();   				break;
			case 0xfb: addb_ex();  				break;
			case 0xfc: ldd_ex();   				break;
			case 0xfd: std_ex();   				break;
			case 0xfe: ldu_ex();   				break;
			case 0xff: stu_ex();   				break;
			}
#else
			(*hd6309_main[hd6309.ireg])();
#endif    /* BIG_SWITCH */

			hd6309_ICount -= cycle_counts_page0[hd6309.ireg];

		} while( hd6309_ICount > 0 );

		hd6309_ICount -= hd6309.extra_cycles;
		hd6309.extra_cycles = 0;
	}

	return cycles - hd6309_ICount;	 /* NS 970908 */
}

HD6309_INLINE void fetch_effective_address( void )
{
	UINT8 postbyte = ROP_ARG(PCD);
	PC++;

	switch(postbyte)
	{
	case 0x00: EA=X;													break;
	case 0x01: EA=X+1;													break;
	case 0x02: EA=X+2;													break;
	case 0x03: EA=X+3;													break;
	case 0x04: EA=X+4;													break;
	case 0x05: EA=X+5;													break;
	case 0x06: EA=X+6;													break;
	case 0x07: EA=X+7;													break;
	case 0x08: EA=X+8;													break;
	case 0x09: EA=X+9;													break;
	case 0x0a: EA=X+10; 												break;
	case 0x0b: EA=X+11; 												break;
	case 0x0c: EA=X+12; 												break;
	case 0x0d: EA=X+13; 												break;
	case 0x0e: EA=X+14; 												break;
	case 0x0f: EA=X+15; 												break;

	case 0x10: EA=X-16; 												break;
	case 0x11: EA=X-15; 												break;
	case 0x12: EA=X-14; 												break;
	case 0x13: EA=X-13; 												break;
	case 0x14: EA=X-12; 												break;
	case 0x15: EA=X-11; 												break;
	case 0x16: EA=X-10; 												break;
	case 0x17: EA=X-9;													break;
	case 0x18: EA=X-8;													break;
	case 0x19: EA=X-7;													break;
	case 0x1a: EA=X-6;													break;
	case 0x1b: EA=X-5;													break;
	case 0x1c: EA=X-4;													break;
	case 0x1d: EA=X-3;													break;
	case 0x1e: EA=X-2;													break;
	case 0x1f: EA=X-1;													break;

	case 0x20: EA=Y;													break;
	case 0x21: EA=Y+1;													break;
	case 0x22: EA=Y+2;													break;
	case 0x23: EA=Y+3;													break;
	case 0x24: EA=Y+4;													break;
	case 0x25: EA=Y+5;													break;
	case 0x26: EA=Y+6;													break;
	case 0x27: EA=Y+7;													break;
	case 0x28: EA=Y+8;													break;
	case 0x29: EA=Y+9;													break;
	case 0x2a: EA=Y+10; 												break;
	case 0x2b: EA=Y+11; 												break;
	case 0x2c: EA=Y+12; 												break;
	case 0x2d: EA=Y+13; 												break;
	case 0x2e: EA=Y+14; 												break;
	case 0x2f: EA=Y+15; 												break;

	case 0x30: EA=Y-16; 												break;
	case 0x31: EA=Y-15; 												break;
	case 0x32: EA=Y-14; 												break;
	case 0x33: EA=Y-13; 												break;
	case 0x34: EA=Y-12; 												break;
	case 0x35: EA=Y-11; 												break;
	case 0x36: EA=Y-10; 												break;
	case 0x37: EA=Y-9;													break;
	case 0x38: EA=Y-8;													break;
	case 0x39: EA=Y-7;													break;
	case 0x3a: EA=Y-6;													break;
	case 0x3b: EA=Y-5;													break;
	case 0x3c: EA=Y-4;													break;
	case 0x3d: EA=Y-3;													break;
	case 0x3e: EA=Y-2;													break;
	case 0x3f: EA=Y-1;													break;

	case 0x40: EA=U;													break;
	case 0x41: EA=U+1;													break;
	case 0x42: EA=U+2;													break;
	case 0x43: EA=U+3;													break;
	case 0x44: EA=U+4;													break;
	case 0x45: EA=U+5;													break;
	case 0x46: EA=U+6;													break;
	case 0x47: EA=U+7;													break;
	case 0x48: EA=U+8;													break;
	case 0x49: EA=U+9;													break;
	case 0x4a: EA=U+10; 												break;
	case 0x4b: EA=U+11; 												break;
	case 0x4c: EA=U+12; 												break;
	case 0x4d: EA=U+13; 												break;
	case 0x4e: EA=U+14; 												break;
	case 0x4f: EA=U+15; 												break;

	case 0x50: EA=U-16; 												break;
	case 0x51: EA=U-15; 												break;
	case 0x52: EA=U-14; 												break;
	case 0x53: EA=U-13; 												break;
	case 0x54: EA=U-12; 												break;
	case 0x55: EA=U-11; 												break;
	case 0x56: EA=U-10; 												break;
	case 0x57: EA=U-9;													break;
	case 0x58: EA=U-8;													break;
	case 0x59: EA=U-7;													break;
	case 0x5a: EA=U-6;													break;
	case 0x5b: EA=U-5;													break;
	case 0x5c: EA=U-4;													break;
	case 0x5d: EA=U-3;													break;
	case 0x5e: EA=U-2;													break;
	case 0x5f: EA=U-1;													break;

	case 0x60: EA=S;													break;
	case 0x61: EA=S+1;													break;
	case 0x62: EA=S+2;													break;
	case 0x63: EA=S+3;													break;
	case 0x64: EA=S+4;													break;
	case 0x65: EA=S+5;													break;
	case 0x66: EA=S+6;													break;
	case 0x67: EA=S+7;													break;
	case 0x68: EA=S+8;													break;
	case 0x69: EA=S+9;													break;
	case 0x6a: EA=S+10; 												break;
	case 0x6b: EA=S+11; 												break;
	case 0x6c: EA=S+12; 												break;
	case 0x6d: EA=S+13; 												break;
	case 0x6e: EA=S+14; 												break;
	case 0x6f: EA=S+15; 												break;

	case 0x70: EA=S-16; 												break;
	case 0x71: EA=S-15; 												break;
	case 0x72: EA=S-14; 												break;
	case 0x73: EA=S-13; 												break;
	case 0x74: EA=S-12; 												break;
	case 0x75: EA=S-11; 												break;
	case 0x76: EA=S-10; 												break;
	case 0x77: EA=S-9;													break;
	case 0x78: EA=S-8;													break;
	case 0x79: EA=S-7;													break;
	case 0x7a: EA=S-6;													break;
	case 0x7b: EA=S-5;													break;
	case 0x7c: EA=S-4;													break;
	case 0x7d: EA=S-3;													break;
	case 0x7e: EA=S-2;													break;
	case 0x7f: EA=S-1;													break;

	case 0x80: EA=X;	X++;											break;
	case 0x81: EA=X;	X+=2;											break;
	case 0x82: X--; 	EA=X;											break;
	case 0x83: X-=2;	EA=X;											break;
	case 0x84: EA=X;													break;
	case 0x85: EA=X+SIGNED(B);											break;
	case 0x86: EA=X+SIGNED(A);											break;
	case 0x87: EA=X+SIGNED(E);											break;
	case 0x88: IMMBYTE(EA); 	EA=X+SIGNED(EA);						break;
	case 0x89: IMMWORD(ea); 	EA+=X;									break;
	case 0x8a: EA=X+SIGNED(F);											break;
	case 0x8b: EA=X+D;													break;
	case 0x8c: IMMBYTE(EA); 	EA=PC+SIGNED(EA);						break;
	case 0x8d: IMMWORD(ea); 	EA+=PC; 								break;
	case 0x8e: EA=X+W;													break;
	case 0x8f: EA=W;		 											break;

	case 0x90: EA=W;								EAD=RM16(EAD);		break;
	case 0x91: EA=X;	X+=2;						EAD=RM16(EAD);		break;
	case 0x92: IIError();												break;
	case 0x93: X-=2;	EA=X;						EAD=RM16(EAD);		break;
	case 0x94: EA=X;								EAD=RM16(EAD);		break;
	case 0x95: EA=X+SIGNED(B);						EAD=RM16(EAD);		break;
	case 0x96: EA=X+SIGNED(A);						EAD=RM16(EAD);		break;
	case 0x97: EA=X+SIGNED(E);						EAD=RM16(EAD);		break;
	case 0x98: IMMBYTE(EA); 	EA=X+SIGNED(EA);	EAD=RM16(EAD);		break;
	case 0x99: IMMWORD(ea); 	EA+=X;				EAD=RM16(EAD);		break;
	case 0x9a: EA=X+SIGNED(F);						EAD=RM16(EAD);		break;
	case 0x9b: EA=X+D;								EAD=RM16(EAD);		break;
	case 0x9c: IMMBYTE(EA); 	EA=PC+SIGNED(EA);	EAD=RM16(EAD);		break;
	case 0x9d: IMMWORD(ea); 	EA+=PC; 			EAD=RM16(EAD);		break;
	case 0x9e: EA=X+W;								EAD=RM16(EAD);		break;
	case 0x9f: IMMWORD(ea); 						EAD=RM16(EAD);		break;

	case 0xa0: EA=Y;	Y++;											break;
	case 0xa1: EA=Y;	Y+=2;											break;
	case 0xa2: Y--; 	EA=Y;											break;
	case 0xa3: Y-=2;	EA=Y;											break;
	case 0xa4: EA=Y;													break;
	case 0xa5: EA=Y+SIGNED(B);											break;
	case 0xa6: EA=Y+SIGNED(A);											break;
	case 0xa7: EA=Y+SIGNED(E);											break;
	case 0xa8: IMMBYTE(EA); 	EA=Y+SIGNED(EA);						break;
	case 0xa9: IMMWORD(ea); 	EA+=Y;									break;
	case 0xaa: EA=Y+SIGNED(F);											break;
	case 0xab: EA=Y+D;													break;
	case 0xac: IMMBYTE(EA); 	EA=PC+SIGNED(EA);						break;
	case 0xad: IMMWORD(ea); 	EA+=PC; 								break;
	case 0xae: EA=Y+W;													break;
	case 0xaf: IMMWORD(ea);     EA+=W;									break;

	case 0xb0: IMMWORD(ea); 	EA+=W;				EAD=RM16(EAD);		break;
	case 0xb1: EA=Y;	Y+=2;						EAD=RM16(EAD);		break;
	case 0xb2: IIError();												break;
	case 0xb3: Y-=2;	EA=Y;						EAD=RM16(EAD);		break;
	case 0xb4: EA=Y;								EAD=RM16(EAD);		break;
	case 0xb5: EA=Y+SIGNED(B);						EAD=RM16(EAD);		break;
	case 0xb6: EA=Y+SIGNED(A);						EAD=RM16(EAD);		break;
	case 0xb7: EA=Y+SIGNED(E);						EAD=RM16(EAD);		break;
	case 0xb8: IMMBYTE(EA); 	EA=Y+SIGNED(EA);	EAD=RM16(EAD);		break;
	case 0xb9: IMMWORD(ea); 	EA+=Y;				EAD=RM16(EAD);		break;
	case 0xba: EA=Y+SIGNED(F);						EAD=RM16(EAD);		break;
	case 0xbb: EA=Y+D;								EAD=RM16(EAD);		break;
	case 0xbc: IMMBYTE(EA); 	EA=PC+SIGNED(EA);	EAD=RM16(EAD);		break;
	case 0xbd: IMMWORD(ea); 	EA+=PC; 			EAD=RM16(EAD);		break;
	case 0xbe: EA=Y+W;								EAD=RM16(EAD);		break;
	case 0xbf: IIError();												break;

	case 0xc0: EA=U;			U++;									break;
	case 0xc1: EA=U;			U+=2;									break;
	case 0xc2: U--; 			EA=U;									break;
	case 0xc3: U-=2;			EA=U;									break;
	case 0xc4: EA=U;													break;
	case 0xc5: EA=U+SIGNED(B);											break;
	case 0xc6: EA=U+SIGNED(A);											break;
	case 0xc7: EA=U+SIGNED(E);											break;
	case 0xc8: IMMBYTE(EA); 	EA=U+SIGNED(EA);						break;
	case 0xc9: IMMWORD(ea); 	EA+=U;									break;
	case 0xca: EA=U+SIGNED(F);											break;
	case 0xcb: EA=U+D;													break;
	case 0xcc: IMMBYTE(EA); 	EA=PC+SIGNED(EA);						break;
	case 0xcd: IMMWORD(ea); 	EA+=PC; 								break;
	case 0xce: EA=U+W;													break;
	case 0xcf: EA=W;            W+=2;									break;

	case 0xd0: EA=W;	W+=2;						EAD=RM16(EAD);		break;
	case 0xd1: EA=U;	U+=2;						EAD=RM16(EAD);		break;
	case 0xd2: IIError();												break;
	case 0xd3: U-=2;	EA=U;						EAD=RM16(EAD);		break;
	case 0xd4: EA=U;								EAD=RM16(EAD);		break;
	case 0xd5: EA=U+SIGNED(B);						EAD=RM16(EAD);		break;
	case 0xd6: EA=U+SIGNED(A);						EAD=RM16(EAD);		break;
	case 0xd7: EA=U+SIGNED(E);						EAD=RM16(EAD);		break;
	case 0xd8: IMMBYTE(EA); 	EA=U+SIGNED(EA);	EAD=RM16(EAD);		break;
	case 0xd9: IMMWORD(ea); 	EA+=U;				EAD=RM16(EAD);		break;
	case 0xda: EA=U+SIGNED(F);						EAD=RM16(EAD);		break;
	case 0xdb: EA=U+D;								EAD=RM16(EAD);		break;
	case 0xdc: IMMBYTE(EA); 	EA=PC+SIGNED(EA);	EAD=RM16(EAD);		break;
	case 0xdd: IMMWORD(ea); 	EA+=PC; 			EAD=RM16(EAD);		break;
	case 0xde: EA=U+W;								EAD=RM16(EAD);		break;
	case 0xdf: IIError();												break;

	case 0xe0: EA=S;	S++;											break;
	case 0xe1: EA=S;	S+=2;											break;
	case 0xe2: S--; 	EA=S;											break;
	case 0xe3: S-=2;	EA=S;											break;
	case 0xe4: EA=S;													break;
	case 0xe5: EA=S+SIGNED(B);											break;
	case 0xe6: EA=S+SIGNED(A);											break;
	case 0xe7: EA=S+SIGNED(E);											break;
	case 0xe8: IMMBYTE(EA); 	EA=S+SIGNED(EA);						break;
	case 0xe9: IMMWORD(ea); 	EA+=S;									break;
	case 0xea: EA=S+SIGNED(F);											break;
	case 0xeb: EA=S+D;													break;
	case 0xec: IMMBYTE(EA); 	EA=PC+SIGNED(EA);						break;
	case 0xed: IMMWORD(ea); 	EA+=PC; 								break;
	case 0xee: EA=S+W;													break;
	case 0xef: W-=2;	EA=W;											break;

	case 0xf0: W-=2;	EA=W;						EAD=RM16(EAD);		break;
	case 0xf1: EA=S;	S+=2;						EAD=RM16(EAD);		break;
	case 0xf2: IIError();												break;
	case 0xf3: S-=2;	EA=S;						EAD=RM16(EAD);		break;
	case 0xf4: EA=S;								EAD=RM16(EAD);		break;
	case 0xf5: EA=S+SIGNED(B);						EAD=RM16(EAD);		break;
	case 0xf6: EA=S+SIGNED(A);						EAD=RM16(EAD);		break;
	case 0xf7: EA=S+SIGNED(E);						EAD=RM16(EAD);		break;
	case 0xf8: IMMBYTE(EA); 	EA=S+SIGNED(EA);	EAD=RM16(EAD);		break;
	case 0xf9: IMMWORD(ea); 	EA+=S;				EAD=RM16(EAD);		break;
	case 0xfa: EA=S+SIGNED(F);						EAD=RM16(EAD);		break;
	case 0xfb: EA=S+D;								EAD=RM16(EAD);		break;
	case 0xfc: IMMBYTE(EA); 	EA=PC+SIGNED(EA);	EAD=RM16(EAD);		break;
	case 0xfd: IMMWORD(ea); 	EA+=PC; 			EAD=RM16(EAD);		break;
	case 0xfe: EA=S+W;								EAD=RM16(EAD);		break;
	case 0xff: IIError();												break;
	}

	hd6309_ICount -= index_cycle[postbyte];
}

#ifdef __cplusplus
}
#endif

#if 0
/**************************************************************************
 * Generic set_info
 **************************************************************************/

static void hd6309_set_info(UINT32 state, cpuinfo *info)
{
	switch (state)
	{
		/* --- the following bits of info are set as 64-bit signed integers --- */
		case CPUINFO_INT_INPUT_STATE + HD6309_IRQ_LINE:	set_irq_line(HD6309_IRQ_LINE, info->i); break;
		case CPUINFO_INT_INPUT_STATE + HD6309_FIRQ_LINE:set_irq_line(HD6309_FIRQ_LINE, info->i); break;
		case CPUINFO_INT_INPUT_STATE + HD6309_INPUT_LINE_NMI:	set_irq_line(HD6309_INPUT_LINE_NMI, info->i);	break;

		case CPUINFO_INT_PC:
		case CPUINFO_INT_REGISTER + HD6309_PC:		PC = info->i; CHANGE_PC;					break;
		case CPUINFO_INT_SP:
		case CPUINFO_INT_REGISTER + HD6309_S:		S = info->i;								break;
		case CPUINFO_INT_REGISTER + HD6309_CC:		CC = info->i; CHECK_IRQ_LINES();			break;
		case CPUINFO_INT_REGISTER + HD6309_MD:		MD = info->i; UpdateState();				break;
		case CPUINFO_INT_REGISTER + HD6309_U: 		U = info->i;								break;
		case CPUINFO_INT_REGISTER + HD6309_A: 		A = info->i;								break;
		case CPUINFO_INT_REGISTER + HD6309_B: 		B = info->i;								break;
		case CPUINFO_INT_REGISTER + HD6309_E: 		E = info->i;								break;
		case CPUINFO_INT_REGISTER + HD6309_F: 		F = info->i;								break;
		case CPUINFO_INT_REGISTER + HD6309_X: 		X = info->i;								break;
		case CPUINFO_INT_REGISTER + HD6309_Y: 		Y = info->i;								break;
		case CPUINFO_INT_REGISTER + HD6309_V: 		V = info->i;								break;
		case CPUINFO_INT_REGISTER + HD6309_DP: 		DP = info->i;								break;
	}
}



/**************************************************************************
 * Generic get_info
 **************************************************************************/

void hd6309_get_info(UINT32 state, cpuinfo *info)
{
	switch (state)
	{
		/* --- the following bits of info are returned as 64-bit signed integers --- */
		case CPUINFO_INT_CONTEXT_SIZE:					info->i = sizeof(hd6309);				break;
		case CPUINFO_INT_INPUT_LINES:					info->i = 2;							break;
		case CPUINFO_INT_DEFAULT_IRQ_VECTOR:			info->i = 0;							break;
		case CPUINFO_INT_ENDIANNESS:					info->i = CPU_IS_BE;					break;
		case CPUINFO_INT_CLOCK_MULTIPLIER:				info->i = 1;							break;
		case CPUINFO_INT_CLOCK_DIVIDER:					info->i = 4;							break;
		case CPUINFO_INT_MIN_INSTRUCTION_BYTES:			info->i = 1;							break;
		case CPUINFO_INT_MAX_INSTRUCTION_BYTES:			info->i = 5;							break;
		case CPUINFO_INT_MIN_CYCLES:					info->i = 1;							break;
		case CPUINFO_INT_MAX_CYCLES:					info->i = 20;							break;

		case CPUINFO_INT_DATABUS_WIDTH + ADDRESS_SPACE_PROGRAM:	info->i = 8;					break;
		case CPUINFO_INT_ADDRBUS_WIDTH + ADDRESS_SPACE_PROGRAM: info->i = 16;					break;
		case CPUINFO_INT_ADDRBUS_SHIFT + ADDRESS_SPACE_PROGRAM: info->i = 0;					break;
		case CPUINFO_INT_DATABUS_WIDTH + ADDRESS_SPACE_DATA:	info->i = 0;					break;
		case CPUINFO_INT_ADDRBUS_WIDTH + ADDRESS_SPACE_DATA: 	info->i = 0;					break;
		case CPUINFO_INT_ADDRBUS_SHIFT + ADDRESS_SPACE_DATA: 	info->i = 0;					break;
		case CPUINFO_INT_DATABUS_WIDTH + ADDRESS_SPACE_IO:		info->i = 0;					break;
		case CPUINFO_INT_ADDRBUS_WIDTH + ADDRESS_SPACE_IO: 		info->i = 0;					break;
		case CPUINFO_INT_ADDRBUS_SHIFT + ADDRESS_SPACE_IO: 		info->i = 0;					break;

		case CPUINFO_INT_INPUT_STATE + HD6309_IRQ_LINE:	info->i = hd6309.irq_state[HD6309_IRQ_LINE]; break;
		case CPUINFO_INT_INPUT_STATE + HD6309_FIRQ_LINE:info->i = hd6309.irq_state[HD6309_FIRQ_LINE]; break;
		case CPUINFO_INT_INPUT_STATE + HD6309_INPUT_LINE_NMI:	info->i = hd6309.nmi_state;				break;

		case CPUINFO_INT_PREVIOUSPC:					info->i = wPPC;							break;

		case CPUINFO_INT_PC:
		case CPUINFO_INT_REGISTER + HD6309_PC:			info->i = PC;							break;
		case CPUINFO_INT_SP:
		case CPUINFO_INT_REGISTER + HD6309_S:			info->i = S;							break;
		case CPUINFO_INT_REGISTER + HD6309_CC:			info->i = CC;							break;
		case CPUINFO_INT_REGISTER + HD6309_MD:			info->i = MD;							break;
		case CPUINFO_INT_REGISTER + HD6309_U:			info->i = U;							break;
		case CPUINFO_INT_REGISTER + HD6309_A:			info->i = A;							break;
		case CPUINFO_INT_REGISTER + HD6309_B:			info->i = B;							break;
		case CPUINFO_INT_REGISTER + HD6309_E:			info->i = E;							break;
		case CPUINFO_INT_REGISTER + HD6309_F:			info->i = F;							break;
		case CPUINFO_INT_REGISTER + HD6309_X:			info->i = X;							break;
		case CPUINFO_INT_REGISTER + HD6309_Y:			info->i = Y;							break;
		case CPUINFO_INT_REGISTER + HD6309_V:			info->i = V;							break;
		case CPUINFO_INT_REGISTER + HD6309_DP:			info->i = DP;							break;

		/* --- the following bits of info are returned as pointers to data or functions --- */
		case CPUINFO_PTR_SET_INFO:						info->setinfo = hd6309_set_info;		break;
		case CPUINFO_PTR_GET_CONTEXT:					info->getcontext = hd6309_get_context;	break;
		case CPUINFO_PTR_SET_CONTEXT:					info->setcontext = hd6309_set_context;	break;
		case CPUINFO_PTR_INIT:							info->init = hd6309_init;				break;
		case CPUINFO_PTR_RESET:							info->reset = hd6309_reset;				break;
		case CPUINFO_PTR_EXIT:							info->exit = hd6309_exit;				break;
		case CPUINFO_PTR_EXECUTE:						info->execute = hd6309_execute;			break;
		case CPUINFO_PTR_BURN:							info->burn = NULL;						break;
		case CPUINFO_PTR_DISASSEMBLE:					info->disassemble = hd6309_dasm;		break;
		case CPUINFO_PTR_INSTRUCTION_COUNTER:			info->icount = &hd6309_ICount;			break;

		/* --- the following bits of info are returned as NULL-terminated strings --- */
		case CPUINFO_STR_NAME:							strcpy(info->s, "HD6309");				break;
		case CPUINFO_STR_CORE_FAMILY:					strcpy(info->s, "Hitachi 6309");		break;
		case CPUINFO_STR_CORE_VERSION:					strcpy(info->s, "1.01");				break;
		case CPUINFO_STR_CORE_FILE:						strcpy(info->s, __FILE__);				break;
		case CPUINFO_STR_CORE_CREDITS:					strcpy(info->s, "Copyright John Butler and Tim Lindner"); break;

		case CPUINFO_STR_FLAGS:
			sprintf(info->s, "%c%c%c%c%c%c%c%c (MD:%c%c%c%c)",
				hd6309.cc & 0x80 ? 'E':'.',
				hd6309.cc & 0x40 ? 'F':'.',
				hd6309.cc & 0x20 ? 'H':'.',
				hd6309.cc & 0x10 ? 'I':'.',
				hd6309.cc & 0x08 ? 'N':'.',
				hd6309.cc & 0x04 ? 'Z':'.',
				hd6309.cc & 0x02 ? 'V':'.',
				hd6309.cc & 0x01 ? 'C':'.',

				hd6309.md & 0x80 ? 'E':'e',
				hd6309.md & 0x40 ? 'F':'f',
				hd6309.md & 0x02 ? 'I':'i',
				hd6309.md & 0x01 ? 'Z':'z');
			break;

		case CPUINFO_STR_REGISTER + HD6309_PC:			sprintf(info->s, "PC:%04X", hd6309.pc.w.l); break;
		case CPUINFO_STR_REGISTER + HD6309_S:			sprintf(info->s, "S:%04X", hd6309.s.w.l); break;
		case CPUINFO_STR_REGISTER + HD6309_CC:			sprintf(info->s, "CC:%02X", hd6309.cc); break;
		case CPUINFO_STR_REGISTER + HD6309_MD:			sprintf(info->s, "MD:%02X", hd6309.md); break;
		case CPUINFO_STR_REGISTER + HD6309_U:			sprintf(info->s, "U:%04X", hd6309.u.w.l); break;
		case CPUINFO_STR_REGISTER + HD6309_A:			sprintf(info->s, "A:%02X", hd6309.d.b.h); break;
		case CPUINFO_STR_REGISTER + HD6309_B:			sprintf(info->s, "B:%02X", hd6309.d.b.l); break;
		case CPUINFO_STR_REGISTER + HD6309_E:			sprintf(info->s, "E:%02X", hd6309.w.b.h); break;
		case CPUINFO_STR_REGISTER + HD6309_F:			sprintf(info->s, "F:%02X", hd6309.w.b.l); break;
		case CPUINFO_STR_REGISTER + HD6309_X:			sprintf(info->s, "X:%04X", hd6309.x.w.l); break;
		case CPUINFO_STR_REGISTER + HD6309_Y:			sprintf(info->s, "Y:%04X", hd6309.y.w.l); break;
		case CPUINFO_STR_REGISTER + HD6309_V:			sprintf(info->s, "V:%04X", hd6309.v.w.l); break;
		case CPUINFO_STR_REGISTER + HD6309_DP:			sprintf(info->s, "DP:%02X", hd6309.dp.b.h); break;
	}
}
#endif
