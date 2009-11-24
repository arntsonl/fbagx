// DirectSound module
#include "burner.h"
#include "aud_dsp.h"
#include <ogcsys.h>
#include <asndlib.h>

//static unsigned char soundbuffer[2][3840] ATTRIBUTE_ALIGN (32);
//static unsigned char mixbuffer[16000];
static unsigned char * soundbuffer; // primary buffer to play sounds
static unsigned char * loopbuffer;	// secondary buffer to play in the background
static int mixhead = 0;
static int mixtail = 0;
static int whichab = 0;
static int IsPlaying = 0;

int (*DSoundGetNextSound)(int);				// Callback used to request more sound

static int cbLoopLen = 0;					// Loop length (in bytes) calculated

/* align_size has to be a power of two !! */
static void *aligned_malloc(size_t size, size_t align_size) {

  char *ptr,*ptr2,*aligned_ptr;
  int align_mask = align_size - 1;

  ptr=(char *)malloc(size + align_size + sizeof(int));
  if(ptr==NULL) return(NULL);

  ptr2 = ptr + sizeof(int);
  aligned_ptr = ptr2 + (align_size - ((size_t)ptr2 & align_mask));


  ptr2 = aligned_ptr - sizeof(int);
  *((int *)ptr2)=(int)(aligned_ptr - ptr);

  return(aligned_ptr);
}

static void aligned_free(void *ptr) {

  int *ptr2=(int *)ptr - 1;
  ptr -= *ptr2;
  free(ptr);
}

static int DSoundGetNextSoundFiller(int)							// int bDraw
{
	if (nAudNextSound == NULL) {
		return 1;
	}
	memset(nAudNextSound, 0, nAudSegLen << 2);						// Write silence into the buffer

	return 0;
}

static int WiiSetCallback(int (*pCallback)(int))
{
	if (pCallback == NULL) {
		DSoundGetNextSound = DSoundGetNextSoundFiller;
	} else {
		DSoundGetNextSound = pCallback;
	}

	return 0;
}

static int WiiBlankSound()
{
	//memset(soundbuffer, 0, 3840*2);
	//memset(mixbuffer, 0, 16000);
	memset(soundbuffer, 0, nAudSegLen);
	//memset(loopbuffer, 0, cbLoopLen); // Loop buffer clear?
	
	mixhead = mixtail = 0;

	// Also blank the nAudNextSound buffer
	if (nAudNextSound) {
		memset(nAudNextSound, 0, nAudSegLen << 2);
	}
	
/*
	void *pData = NULL, *pData2 = NULL;
	DWORD cbLen = 0, cbLen2 = 0;

	// Lock the Loop buffer
	if (FAILED(pdsbLoop->Lock(0, cbLoopLen, &pData, &cbLen, &pData2, &cbLen2, 0))) {
		return 1;
	}
	memset(pData, 0, cbLen);

	// Unlock (2nd 0 is because we wrote nothing to second part)
	pdsbLoop->Unlock(pData, cbLen, pData2, 0);

	// Also blank the nAudNextSound buffer
	if (nAudNextSound) {
		memset(nAudNextSound, 0, nAudSegLen << 2);
	}
*/
	return 0;
}

static int nDSoundNextSeg = 0;										// We have filled the sound in the loop up to the beginning of 'nNextSeg'

#define WRAP_INC(x) { x++; if (x >= nAudSegCount) x = 0; }

// This function checks the DSound loop, and if necessary does a callback to update the emulation
static int WiiSoundCheck()
{
	int nPlaySeg = 0, nFollowingSeg;

	// Get the current position?
	nPlaySeg = 0;  // pdsbLoop->GetCurrentPosition
	if ( nPlaySeg > nAudSegCount -1 ){
		nPlaySeg = nAudSegCount - 1;
	}
	if (nPlaySeg < 0 ){
		nPlaySeg = 0;
	}
	
	if (nDSoundNextSeg == nPlaySeg) {
		//Sleep(2);													// Don't need to do anything for a bit

		return 0;
	}

	// work out which seg we will fill next
	nFollowingSeg = nDSoundNextSeg;
	WRAP_INC(nFollowingSeg);

	while (nDSoundNextSeg != nPlaySeg) {
		void *pData = NULL, *pData2 = NULL;
		int bDraw;

		// copy the data to pData?
		
		// fill nNextSeg

		// Lock the relevant seg of the loop buffer
		//if (SUCCEEDED(pdsbLoop->Lock(nDSoundNextSeg * (nAudSegLen << 2), nAudSegLen << 2, &pData, &cbLen, &pData2, &cbLen2, 0))) {
		// Locked the segment, so write the sound we calculated last time
		//memcpy(pData, nAudNextSound, nAudSegLen << 2);

			// Unlock (2nd 0 is because we wrote nothing to second part)
		//	pdsbLoop->Unlock(pData, cbLen, pData2, 0);
	//	}

		memcpy(loopbuffer, nAudNextSound, nAudSegLen << 2);
	
		bDraw = (nFollowingSeg == nPlaySeg)	|| bAlwaysDrawFrames;	// If this is the last seg of sound, flag bDraw (to draw the graphics)

		DSoundGetNextSound(bDraw);									// get more sound into nAudNextSound

		if (nAudDSPModule[0])	{
			DspDo(nAudNextSound, nAudSegLen);
		}

		nDSoundNextSeg = nFollowingSeg;
		WRAP_INC(nFollowingSeg);
	}
	return 0;
}

static int WiiSoundExit()
{
	AUDIO_StopDMA();

	DspExit();

	free(nAudNextSound);
	nAudNextSound = NULL;

	DSoundGetNextSound = NULL;
	
	aligned_free(soundbuffer);
	aligned_free(loopbuffer);
	
	return 0;
}

static int WiiSoundInit()
{
	ASND_Init(); // does this break anything to call twice?

	if (nAudSampleRate <= 0) {
		return 1;
	}
	
	nAudSegLen = (nAudSampleRate[0] * 100 + (6000 >> 1)) / 6000; // this is how big our audio segment should be?
	cbLoopLen = (nAudSegLen * nAudSegCount) << 2;				 // how many audio segments can we have?
	
	//memset(soundbuffer, 0, 3840*2);
	//memset(mixbuffer, 0, 16000);
	soundbuffer = (unsigned char*)aligned_malloc(nAudSegLen << 2, 32);	// Align 32 for the main sound
	loopbuffer = (unsigned char*)aligned_malloc(cbLoopLen, 32);			// Align 32 for the loop buffer sound
	
	nAudNextSound = (short*)malloc(nAudSegLen << 2);		// The next sound block to put in the stream
	if (nAudNextSound == NULL) {
		WiiSoundExit();
		return 1;
	}
	
	WiiSetCallback(NULL);
	
	DspInit();
	/*
	int nRet = 0;
	DSBUFFERDESC dsbd;
	WAVEFORMATEX wfx;

	if (nAudSampleRate <= 0) {
		return 1;
	}

	nDSoundFps = nAppVirtualFps;

	// Calculate the Seg Length and Loop length (round to nearest sample)
	nAudSegLen = (nAudSampleRate[0] * 100 + (nDSoundFps >> 1)) / nDSoundFps;
	cbLoopLen = (nAudSegLen * nAudSegCount) << 2;

	// Make the format of the sound
	memset(&wfx, 0, sizeof(wfx));
	wfx.cbSize = sizeof(wfx);
	wfx.wFormatTag = WAVE_FORMAT_PCM;
	wfx.nChannels = 2;										  // stereo
	wfx.nSamplesPerSec = nAudSampleRate[0];					  // sample rate
	wfx.wBitsPerSample = 16;								  // 16-bit
	wfx.nBlockAlign = 4;									  // bytes per sample
	wfx.nAvgBytesPerSec = wfx.nSamplesPerSec * wfx.nBlockAlign;

	// Create the DirectSound interface
	if (FAILED(DirectSoundCreate(NULL, &pDS, NULL))) {
		return 1;
	}

	// Set the coop level
	nRet = pDS->SetCooperativeLevel(hScrnWnd, DSSCL_PRIORITY);

	// Make the primary sound buffer
	memset(&dsbd, 0, sizeof(dsbd));
	dsbd.dwSize = sizeof(dsbd);
	dsbd.dwFlags = DSBCAPS_PRIMARYBUFFER;
	if (FAILED(pDS->CreateSoundBuffer(&dsbd, &pdsbPrim, NULL))) {
		DxSoundExit();
		return 1;
	}

	{
		// Set the format of the primary sound buffer (not critical if it fails)
		if (nAudSampleRate[0] < 44100) {
			wfx.nSamplesPerSec = 44100;
		}
		pdsbPrim->SetFormat(&wfx);

		wfx.nSamplesPerSec = nAudSampleRate[0];
	}

	// Make the loop sound buffer
	memset(&dsbd, 0, sizeof(dsbd));
	dsbd.dwSize = sizeof(dsbd);
	// A standard secondary buffer (accurate position, plays in the background, and can notify).
	dsbd.dwFlags = DSBCAPS_GETCURRENTPOSITION2 | DSBCAPS_GLOBALFOCUS | DSBCAPS_CTRLPOSITIONNOTIFY | DSBCAPS_CTRLVOLUME;
	dsbd.dwBufferBytes = cbLoopLen;
	dsbd.lpwfxFormat = &wfx;								// Same format as the primary buffer
	if (FAILED(pDS->CreateSoundBuffer(&dsbd, &pdsbLoop, NULL))) {
		AudSoundExit();
		return 1;
	}

	nAudNextSound = (short*)malloc(nAudSegLen << 2);		// The next sound block to put in the stream
	if (nAudNextSound == NULL) {
		DxSoundExit();
		return 1;
	}

	DxSetCallback(NULL);

	DspInit();
	*/
	return 0;
}

static int WiiSoundPlay()
{
	/*
	DxBlankSound();
	pdsbLoop->SetVolume(nDSoundVol);

	// Play the looping buffer
	if (FAILED(pdsbLoop->Play(0, 0, DSBPLAY_LOOPING))) {
		return 1;
	}
	*/
	
	WiiBlankSound(); // why do this?
	// Set Volume
	
	// Play the Sound?
	DCFlushRange(loopbuffer, cbLoopLen+(cbLoopLen%32));
	AUDIO_InitDMA((u32)loopbuffer, cbLoopLen+(cbLoopLen%32));
	AUDIO_StartDMA();
	
	bAudPlaying = 1;

	return 0;
}

static int WiiSoundStop()
{
	bAudPlaying = 0;

	if (bAudOkay == 0) {
		return 1;
	}

	ASND_Pause(1); // no need for DMA switching yet
	
	return 0;
}

static int WiiSoundSetVolume()
{
	if (nAudVolume == 10000) {
		//nDSoundVol = DSBVOLUME_MAX;
	} 
	else {
		if (nAudVolume == 0) {
			//nDSoundVol = DSBVOLUME_MIN;
		} 
		else {
			//nDSoundVol = DSBVOLUME_MAX - (long)(10000.0 * pow(10.0, nAudVolume / -5000.0)) + 100;
		}
	}
	
	return 1;
}

static int WiiGetSettings(InterfaceInfo* pInfo)
{
/*
	TCHAR szString[MAX_PATH] = _T("");

	_sntprintf(szString, MAX_PATH, _T("Audio is delayed by approx. %ims"), int(100000.0 / (nDSoundFps / (nAudSegCount - 1.0))));
	IntInfoAddStringModule(pInfo, szString);
	*/
	return 0;
}

struct AudOut AudOutWii = { WiiBlankSound, WiiSoundCheck, WiiSoundInit, WiiSetCallback, WiiSoundPlay, WiiSoundStop, WiiSoundExit, WiiSoundSetVolume, WiiGetSettings, _T("Wii Sound audio output") };
