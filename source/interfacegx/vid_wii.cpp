// Wii blitter
#include "burner.h"

static int nRotateGame;

static int nGameWidth = 0, nGameHeight = 0;		// screen size

static RECT Src = { 0, 0, 0, 0 };
static RECT Dest = { 0, 0, 0, 0 };

static int nHalfMask = 0;

static int nUseSys;								// Use System or Video memory

static int vidExit()
{
	return 0;
}

static int vidInit()
{
	nGameWidth = nVidImageWidth; nGameHeight = nVidImageHeight;

	nRotateGame = 0;
	
	if (bDrvOkay) {
		// Get the game screen size
		BurnDrvGetVisibleSize(&nGameWidth, &nGameHeight);

	    if (BurnDrvGetFlags() & BDF_ORIENTATION_VERTICAL) {
			if (nVidRotationAdjust & 1) {
				int n = nGameWidth;
				nGameWidth = nGameHeight;
				nGameHeight = n;
				nRotateGame |= (nVidRotationAdjust & 2);
			} else {
				nRotateGame |= 1;
			}
		}
		if (BurnDrvGetFlags() & BDF_ORIENTATION_FLIPPED) {
			nRotateGame ^= 2;
		}
		
		// No rotation for now
	}

	return 0;
}

// Run one frame and render the screen
int vidFrame(bool bRedraw)			// bRedraw = 0
{
	if (pVidImage == NULL) {
		return 1;
	}

	if (bDrvOkay) {
		if (bRedraw) {				// Redraw current frame
			if (BurnDrvRedraw()) {
				BurnDrvFrame();		// No redraw function provided, advance one frame
			}
		} else {
			BurnDrvFrame();			// Run one frame and draw the screen
		}
	}

	return 0;
}

static int vidPaint(int bValidate)
{
	// Do some painting
	return 0;
}

static int vidScale(RECT* pRect, int nWidth, int nHeight)
{
	return 0;
}

static int vidGetSettings(InterfaceInfo* pInfo)
{
	return 0;
}

// The video output plugin:
struct VidOut VidOutGX = { vidInit, vidExit, vidFrame, vidPaint, vidScale, vidGetSettings, _T("Wii video output") };
