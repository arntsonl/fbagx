#include "burnint.h"
#include "burn_sound.h"

short Precalc[4096 *4];

// Routine used to precalculate the table used for interpolation
int cmc_4p_Precalc()
{
	int a, x, x2, x3;

	for (a = 0; a < 4096; a++) {
		x  = a  * 4;			// x = 0..16384
		x2 = x  * x / 16384;	// pow(x, 2);
		x3 = x2 * x / 16384;	// pow(x, 3);

		Precalc[a * 4 + 0] = (short)(-x / 3 + x2 / 2 - x3 / 6);
		Precalc[a * 4 + 1] = (short)(-x / 2 - x2     + x3 / 2 + 16384);
		Precalc[a * 4 + 2] = (short)( x     + x2 / 2 - x3 / 2);
		Precalc[a * 4 + 3] = (short)(-x / 6 + x3 / 6);
	}

	return 0;
}

// Originally defined in our burn_sound_a.asm, need to convert this
int ChannelMix_QS_A(int* Dest, int nLen,
								char* Sample, int LoopEnd,
								int* Pos,
								int VolL, int VolR,
								int LoopLen,
								int IncPos,
								char* EndBuff)
{
	// not defined currently
	return -1;
}

void BurnSoundCopyClamp_A(int* Src, short* Dest, int Len)
{
	// not defined currently
	return;
}
	
void BurnSoundCopyClamp_Add_A(int* Src, short* Dest, int Len)
{
	// not defined currently
	return;
}
	
void BurnSoundCopyClamp_Mono_A(int* Src, short* Dest, int Len)
{
	// not defined currently
	return;
}
	
void BurnSoundCopyClamp_Mono_Add_A(int* Src, short* Dest, int Len)
{
	// not defined currently
	return;
}
	
void BurnSoundCopy_FM_A(short* SrcL, short* SrcR, short* Dest, int Len, int VolL, int VolR)
{
	// not defined currently
	return;
}
	
void BurnSoundCopy_FM_Add_A(short* SrcL, short* SrcR, short* Dest, int Len, int VolL, int VolR)
{
	// not defined currently
	return;
}

/* SrcOPN should have left channel data at SrcOPN, right channel at SrcOPN + 4096, SrcPSG should have all summed channels */
void BurnSoundCopy_FM_OPN_A(short* SrcOPN, int* SrcPSG, short* Dest, int Len, int VolPSGL, int VolPSGR)
{
	// not defined currently
	return;
}
	
void BurnSoundCopy_FM_OPN_Add_A(short* SrcOPN, int* SrcPSG, short* Dest, int Len, int VolPSGL, int VolPSGR)
{
	// not defined currently
	return;
}

