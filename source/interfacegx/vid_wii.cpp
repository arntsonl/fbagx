// Wii blitter
#include "burner.h"
#include "video.h"
#include <ogcsys.h>
#include "vid_support.h"

static int nRotateGame;

static int nGameWidth = 0, nGameHeight = 0;		// screen size

// GX goodness taken from Snes9xgx (thanks Tantric!)
/*** 2D Video ***/
static unsigned int *xfb[2] = { NULL, NULL }; // Double buffered
static int whichfb = 0; // Switch
static GXRModeObj *vmode = NULL; // Current video mode
static int oldRenderMode = -1; // set to GCSettings.render when changing (temporarily) to another mode
int CheckVideo = 0; // for forcing video reset

/*** GX ***/
#define TEX_WIDTH 1024
#define TEX_HEIGHT 1024
#define TEXTUREMEM_SIZE 	TEX_WIDTH*(TEX_HEIGHT+8)*2
static unsigned char texturemem[TEXTUREMEM_SIZE] ATTRIBUTE_ALIGN (32);

#define DEFAULT_FIFO_SIZE 256 * 1024
static unsigned int copynow = GX_FALSE;
static unsigned char gp_fifo[DEFAULT_FIFO_SIZE] ATTRIBUTE_ALIGN (32);
static GXTexObj texobj;
static Mtx view;
static Mtx GXmodelView2D;

u8 * gameScreenTex = NULL; // a GX texture screen capture of the game
u8 * gameScreenTex2 = NULL; // a GX texture screen capture of the game (copy)

u8 vmode_60hz = 0;
int timerstyle = 0;
bool progressive = 0;

#define HASPECT 320
#define VASPECT 240

/* New texture based scaler */
typedef struct tagcamera
{
	guVector pos;
	guVector up;
	guVector view;
}
camera;

/*** Square Matrix
     This structure controls the size of the image on the screen.
	 Think of the output as a -80 x 80 by -60 x 60 graph.
***/
s16 square[] ATTRIBUTE_ALIGN (32) =
{
  /*
   * X,   Y,  Z
   * Values set are for roughly 4:3 aspect
   */
	-HASPECT,  VASPECT, 0,		// 0
	 HASPECT,  VASPECT, 0,	// 1
	 HASPECT, -VASPECT, 0,	// 2
	-HASPECT, -VASPECT, 0	// 3
};


static camera cam = {
	{0.0F, 0.0F, 0.0F},
	{0.0F, 0.5F, 0.0F},
	{0.0F, 0.0F, -0.5F}
};


/***
*** Custom Video modes (used to emulate original console video modes)
***/

/** Original SNES PAL Resolutions: **/

/* 239 lines progressive (PAL 50Hz) */
static GXRModeObj TV_239p =
{
	VI_TVMODE_PAL_DS,       // viDisplayMode
	512,             // fbWidth
	239,             // efbHeight
	239,             // xfbHeight
	(VI_MAX_WIDTH_PAL - 640)/2,         // viXOrigin
	(VI_MAX_HEIGHT_PAL/2 - 478/2)/2,        // viYOrigin
	640,             // viWidth
	478,             // viHeight
	VI_XFBMODE_SF,   // xFBmode
	GX_FALSE,        // field_rendering
	GX_FALSE,        // aa

	// sample points arranged in increasing Y order
	{
		{6,6},{6,6},{6,6},  // pix 0, 3 sample points, 1/12 units, 4 bits each
		{6,6},{6,6},{6,6},  // pix 1
		{6,6},{6,6},{6,6},  // pix 2
		{6,6},{6,6},{6,6}   // pix 3
	},

	// vertical filter[7], 1/64 units, 6 bits each
	{
		0,         // line n-1
		0,         // line n-1
		21,         // line n
		22,         // line n
		21,         // line n
		0,         // line n+1
		0          // line n+1
	}
};

/* 478 lines interlaced (PAL 50Hz, Deflicker) */
static GXRModeObj TV_478i =
{
	VI_TVMODE_PAL_INT,      // viDisplayMode
	512,             // fbWidth
	478,             // efbHeight
	478,             // xfbHeight
	(VI_MAX_WIDTH_PAL - 640)/2,         // viXOrigin
	(VI_MAX_HEIGHT_PAL - 478)/2,        // viYOrigin
	640,             // viWidth
	478,             // viHeight
	VI_XFBMODE_DF,   // xFBmode
	GX_FALSE,         // field_rendering
	GX_FALSE,        // aa

	// sample points arranged in increasing Y order
	{
		{6,6},{6,6},{6,6},  // pix 0, 3 sample points, 1/12 units, 4 bits each
		{6,6},{6,6},{6,6},  // pix 1
		{6,6},{6,6},{6,6},  // pix 2
		{6,6},{6,6},{6,6}   // pix 3
	},

	// vertical filter[7], 1/64 units, 6 bits each
	{
		8,         // line n-1
		8,         // line n-1
		10,         // line n
		12,         // line n
		10,         // line n
		8,         // line n+1
		8          // line n+1
	}
};

/** Original SNES NTSC Resolutions: **/

/* 224 lines progressive (NTSC or PAL 60Hz) */
static GXRModeObj TV_224p =
{
	VI_TVMODE_EURGB60_DS,      // viDisplayMode
	512,             // fbWidth
	224,             // efbHeight
	224,             // xfbHeight
	(VI_MAX_WIDTH_NTSC - 640)/2,	// viXOrigin
	(VI_MAX_HEIGHT_NTSC/2 - 448/2)/2,	// viYOrigin
	640,             // viWidth
	448,             // viHeight
	VI_XFBMODE_SF,   // xFBmode
	GX_FALSE,        // field_rendering
	GX_FALSE,        // aa

	// sample points arranged in increasing Y order
	{
		{6,6},{6,6},{6,6},  // pix 0, 3 sample points, 1/12 units, 4 bits each
		{6,6},{6,6},{6,6},  // pix 1
		{6,6},{6,6},{6,6},  // pix 2
		{6,6},{6,6},{6,6}   // pix 3
	},

	// vertical filter[7], 1/64 units, 6 bits each
	{
		0,         // line n-1
		0,         // line n-1
		21,         // line n
		22,         // line n
		21,         // line n
		0,         // line n+1
		0          // line n+1
	}
};

/* 448 lines interlaced (NTSC or PAL 60Hz, Deflicker) */
static GXRModeObj TV_448i =
{
	VI_TVMODE_EURGB60_INT,     // viDisplayMode
	512,             // fbWidth
	448,             // efbHeight
	448,             // xfbHeight
	(VI_MAX_WIDTH_NTSC - 640)/2,        // viXOrigin
	(VI_MAX_HEIGHT_NTSC - 448)/2,       // viYOrigin
	640,             // viWidth
	448,             // viHeight
	VI_XFBMODE_DF,   // xFBmode
	GX_FALSE,         // field_rendering
	GX_FALSE,        // aa


	// sample points arranged in increasing Y order
	{
		{6,6},{6,6},{6,6},  // pix 0, 3 sample points, 1/12 units, 4 bits each
		{6,6},{6,6},{6,6},  // pix 1
		{6,6},{6,6},{6,6},  // pix 2
		{6,6},{6,6},{6,6}   // pix 3
	},

	// vertical filter[7], 1/64 units, 6 bits each
	{
		8,         // line n-1
		8,         // line n-1
		10,         // line n
		12,         // line n
		10,         // line n
		8,         // line n+1
		8          // line n+1
	}
};

static GXRModeObj TV_Custom;

/* TV Modes table */
static GXRModeObj *tvmodes[4] = {
	&TV_239p, &TV_478i,			/* Snes PAL video modes */
	&TV_224p, &TV_448i,			/* Snes NTSC video modes */
};


/****************************************************************************
 * copy_to_xfb
 *
 * Stock code to copy the GX buffer to the current display mode.
 * Also increments the frameticker, as it's called for each vb.
 ***************************************************************************/
static inline void
copy_to_xfb (u32 arg)
{
	if (copynow == GX_TRUE)
	{
		GX_CopyDisp (xfb[whichfb], GX_TRUE);
		GX_Flush ();
		copynow = GX_FALSE;
	}
	FrameTimer++;
}

/****************************************************************************
 * Scaler Support Functions
 ***************************************************************************/
static inline void
draw_init ()
{
	GX_ClearVtxDesc ();
	GX_SetVtxDesc (GX_VA_POS, GX_INDEX8);
	GX_SetVtxDesc (GX_VA_CLR0, GX_INDEX8);
	GX_SetVtxDesc (GX_VA_TEX0, GX_DIRECT);

	GX_SetVtxAttrFmt (GX_VTXFMT0, GX_VA_POS, GX_POS_XYZ, GX_S16, 0);
	GX_SetVtxAttrFmt (GX_VTXFMT0, GX_VA_CLR0, GX_CLR_RGBA, GX_RGBA8, 0);
	GX_SetVtxAttrFmt (GX_VTXFMT0, GX_VA_TEX0, GX_TEX_ST, GX_F32, 0);

	GX_SetArray (GX_VA_POS, square, 3 * sizeof (s16));

	GX_SetNumTexGens (1);
	GX_SetNumChans (0);

	GX_SetTexCoordGen (GX_TEXCOORD0, GX_TG_MTX2x4, GX_TG_TEX0, GX_IDENTITY);

	GX_SetTevOp (GX_TEVSTAGE0, GX_REPLACE);
	GX_SetTevOrder (GX_TEVSTAGE0, GX_TEXCOORD0, GX_TEXMAP0, GX_COLORNULL);

	memset (&view, 0, sizeof (Mtx));
	guLookAt(view, &cam.pos, &cam.up, &cam.view);
	GX_LoadPosMtxImm (view, GX_PNMTX0);

	GX_InvVtxCache ();	// update vertex cache
}

static inline void
draw_vert (u8 pos, u8 c, f32 s, f32 t)
{
	GX_Position1x8 (pos);
	GX_Color1x8 (c);
	GX_TexCoord2f32 (s, t);
}

static inline void
draw_square (Mtx v)
{
	Mtx m;			// model matrix.
	Mtx mv;			// modelview matrix.

	guMtxIdentity (m);
	guMtxTransApply (m, m, 0, 0, -100);
	guMtxConcat (v, m, mv);

	GX_LoadPosMtxImm (mv, GX_PNMTX0);
	GX_Begin (GX_QUADS, GX_VTXFMT0, 4);
	draw_vert (0, 0, 0.0, 0.0);
	draw_vert (1, 0, 1.0, 0.0);
	draw_vert (2, 0, 1.0, 1.0);
	draw_vert (3, 0, 0.0, 1.0);
	GX_End ();
}

/****************************************************************************
 * FindVideoMode
 *
 * Finds the optimal video mode, or uses the user-specified one
 * Also configures original video modes
 ***************************************************************************/
static GXRModeObj * FindVideoMode()
{
	GXRModeObj * mode;
	
	// choose the desired video mode
	switch(0) // default it for now
	{
		case 1: // NTSC (480i)
			mode = &TVNtsc480IntDf;
			break;
		case 2: // Progressive (480p)
			mode = &TVNtsc480Prog;
			break;
		case 3: // PAL (50Hz)
			mode = &TVPal574IntDfScale;
			break;
		case 4: // PAL (60Hz)
			mode = &TVEurgb60Hz480IntDf;
			break;
		default:
			mode = VIDEO_GetPreferredMode(NULL);

			/* we have component cables, but the preferred mode is interlaced
			 * why don't we switch into progressive?
			 * on the Wii, the user can do this themselves on their Wii Settings */
			if(VIDEO_HaveComponentCable())
				mode = &TVNtsc480Prog;

			// use hardware vertical scaling to fill screen
			if(mode->viTVMode >> 2 == VI_PAL)
				mode = &TVPal574IntDfScale;
			break;
	}

	// configure original modes
	switch (mode->viTVMode >> 2)
	{
		case VI_PAL:
			// 576 lines (PAL 50Hz)
			vmode_60hz = 0;

			// Original Video modes (forced to PAL 50Hz)
			// set video signal mode
			TV_224p.viTVMode = VI_TVMODE_PAL_DS;
			TV_448i.viTVMode = VI_TVMODE_PAL_INT;
			// set VI position
			TV_224p.viYOrigin = (VI_MAX_HEIGHT_PAL/2 - 448/2)/2;
			TV_448i.viYOrigin = (VI_MAX_HEIGHT_PAL - 448)/2;
			break;

		case VI_NTSC:
			// 480 lines (NTSC 60Hz)
			vmode_60hz = 1;

			// Original Video modes (forced to NTSC 60hz)
			// set video signal mode
			TV_239p.viTVMode = VI_TVMODE_NTSC_DS;
			TV_478i.viTVMode = VI_TVMODE_NTSC_INT;
			TV_224p.viTVMode = VI_TVMODE_NTSC_DS;
			TV_448i.viTVMode = VI_TVMODE_NTSC_INT;
			// set VI position
			TV_239p.viYOrigin = (VI_MAX_HEIGHT_NTSC/2 - 478/2)/2;
			TV_478i.viYOrigin = (VI_MAX_HEIGHT_NTSC - 478)/2;
			TV_224p.viYOrigin = (VI_MAX_HEIGHT_NTSC/2 - 448/2)/2;
			TV_448i.viYOrigin = (VI_MAX_HEIGHT_NTSC - 448)/2;
			break;

		default:
			// 480 lines (PAL 60Hz)
			vmode_60hz = 1;

			// Original Video modes (forced to PAL 60hz)
			// set video signal mode
			TV_239p.viTVMode = VI_TVMODE(mode->viTVMode >> 2, VI_NON_INTERLACE);
			TV_478i.viTVMode = VI_TVMODE(mode->viTVMode >> 2, VI_INTERLACE);
			TV_224p.viTVMode = VI_TVMODE(mode->viTVMode >> 2, VI_NON_INTERLACE);
			TV_448i.viTVMode = VI_TVMODE(mode->viTVMode >> 2, VI_INTERLACE);
			// set VI position
			TV_239p.viYOrigin = (VI_MAX_HEIGHT_NTSC/2 - 478/2)/2;
			TV_478i.viYOrigin = (VI_MAX_HEIGHT_NTSC - 478)/2;
			TV_224p.viYOrigin = (VI_MAX_HEIGHT_NTSC/2 - 448/2)/2;
			TV_448i.viYOrigin = (VI_MAX_HEIGHT_NTSC - 448)/2;
			break;
	}

	// check for progressive scan
	if (mode->viTVMode == VI_TVMODE_NTSC_PROG)
		progressive = true;
	else
		progressive = false;

	// widescreen fix
	if(CONF_GetAspectRatio() == CONF_ASPECT_16_9)
	{
		mode->viWidth = VI_MAX_WIDTH_PAL;
	}
	return mode;
}

/****************************************************************************
 * SetupVideoMode
 *
 * Sets up the given video mode
 ***************************************************************************/
static void SetupVideoMode(GXRModeObj * mode)
{
	if(vmode == mode)
		return;
	
	VIDEO_SetPostRetraceCallback (NULL);
	copynow = GX_FALSE;
	VIDEO_Configure (mode);
	VIDEO_Flush();

	// Allocate the video buffers
	if(xfb[0]) free(MEM_K1_TO_K0(xfb[0]));
	if(xfb[1]) free(MEM_K1_TO_K0(xfb[1]));
	xfb[0] = (u32 *) MEM_K0_TO_K1 (SYS_AllocateFramebuffer (mode));
	xfb[1] = (u32 *) MEM_K0_TO_K1 (SYS_AllocateFramebuffer (mode));

	// Clear framebuffers etc.
	VIDEO_ClearFrameBuffer (mode, xfb[0], COLOR_BLACK);
	VIDEO_ClearFrameBuffer (mode, xfb[1], COLOR_BLACK);
	VIDEO_SetNextFramebuffer (xfb[0]);

	VIDEO_SetBlack (FALSE);
	VIDEO_Flush ();
	VIDEO_WaitVSync ();
		
	if (mode->viTVMode & VI_NON_INTERLACE)
		VIDEO_WaitVSync();
	else
		while (VIDEO_GetNextField())
			VIDEO_WaitVSync();
	
	VIDEO_SetPostRetraceCallback ((VIRetraceCallback)copy_to_xfb);
	vmode = mode;
}

/****************************************************************************
 * MakeTexture
 *
 * Modified for a buffer with an offset (border)
 ***************************************************************************/
void
MakeTexture (const void *src, void *dst, s32 width, s32 height)
{
	register u32 tmp0=0,tmp1=0,tmp2=0,tmp3=0;

	__asm__ __volatile__ (
		"	srwi		%6,%6,2\n"
		"	srwi		%7,%7,2\n"
		"	subi		%3,%4,4\n"
		"	mr			%4,%3\n"
		"	subi		%4,%4,4\n"

		"2: mtctr		%6\n"
		"	mr			%0,%5\n"
		//
		"1: lwz			%1,0(%5)\n"			//1
		"	stwu		%1,8(%4)\n"
		"	lwz			%2,4(%5)\n"			//1
		"	stwu		%2,8(%3)\n"
		"	lwz			%1,1032(%5)\n"		//2
		"	stwu		%1,8(%4)\n"
		"	lwz			%2,1036(%5)\n"		//2
		"	stwu		%2,8(%3)\n"
		"	lwz			%1,2064(%5)\n"		//3
		"	stwu		%1,8(%4)\n"
		"	lwz			%2,2068(%5)\n"		//3
		"	stwu		%2,8(%3)\n"
		"	lwz			%1,3096(%5)\n"		//4
		"	stwu		%1,8(%4)\n"
		"	lwz			%2,3100(%5)\n"		//4
		"	stwu		%2,8(%3)\n"
		"	addi		%5,%5,8\n"
		"	bdnz		1b\n"
		"	addi		%5,%0,4128\n"		//5
		"	subic.		%7,%7,1\n"
		"	bne			2b"
		//		0			 1			  2			   3		   4		  5		    6		    7
		: "=&b"(tmp0), "=&b"(tmp1), "=&b"(tmp2), "=&b"(tmp3), "+b"(dst) : "b"(src), "b"(width), "b"(height)
	);
}

static int vidExit()
{
	return 0;
}

static int vidInit()
{
	VIDEO_Init(); // maybe?
	GXRModeObj *rmode = FindVideoMode();
	SetupVideoMode(rmode);
	//InitLUTs();	// init LUTs for hq2x
	//LWP_CreateThread (&vbthread, vbgetback, NULL, vbstack, TSTACK, 68);
	
	// Initialize GX
	GXColor background = { 0, 0, 0, 0xff };
	memset (&gp_fifo, 0, DEFAULT_FIFO_SIZE);
	GX_Init (&gp_fifo, DEFAULT_FIFO_SIZE);
	GX_SetCopyClear (background, 0x00ffffff);
	GX_SetDispCopyGamma (GX_GM_1_0);
	GX_SetCullMode (GX_CULL_NONE);

	draw_init(); // init the drawing calls (sort of?)
	
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
		// C32 - Maybe init video texture here?
		// initialize the texture obj we are going to use
		GX_InitTexObj (&texobj, texturemem, nGameWidth, nGameHeight, GX_TF_RGBA8, GX_CLAMP, GX_CLAMP, GX_FALSE);
	    GX_InitTexObjLOD(&texobj,GX_NEAR,GX_NEAR_MIP_NEAR,2.5,9.0,0.0,GX_FALSE,GX_FALSE,GX_ANISO_1); // original/unfiltered video mode: force texture filtering OFF
		GX_LoadTexObj (&texobj, GX_TEXMAP0);	// load texture object so its ready to use

	}
	
	// Init the buffer surfaces
	if (nRotateGame & 1) {
		nVidImageWidth = nGameHeight;
		nVidImageHeight = nGameWidth;
	} else {
		nVidImageWidth = nGameWidth;
		nVidImageHeight = nGameHeight;
	}

	nVidImageDepth = 32;	// Get color depth of primary surface (pretend we're running 32-bit)
	nVidImageBPP = (nVidImageDepth + 7) >> 3;
	
	// Make the normal memory buffer
	if (VidSAllocVidImage()) {
		return 1;
	}
	
	// Use our callback to get colors:
	SetBurnHighCol(nVidImageDepth);

	return 0;
}

static int vidBurnToSurface()
{
	whichfb ^= 1;
	//MakeTexture((char *) GFX.Screen, (char *) texturemem, 640, 480);
	unsigned char *pd, *ps, *pdd;
	unsigned char *Surf;
	int nPitch;
	
	pd = texturemem;
	switch (nVidImageBPP) {
		case 4: {
			for (int y = 0; y < nGameHeight; y++, pd += nPitch) {
				ps = pVidImage + (nGameHeight - 1 - y) * 4;
				pdd = pd;
				for (int x = 0; x < nGameWidth; x++) {
					*(int*)pdd = *(int*)ps;
					ps += nVidImagePitch;
					pdd += 4;
				}
			}
			break;
		}
		case 3: {
			for (int y = 0; y < nGameHeight; y++, pd += nPitch) {
				ps = pVidImage + (nGameHeight - 1 - y) * 3;
				pdd = pd;
				for (int x = 0; x < nGameWidth; x++) {
					pdd[0] = ps[0];
					pdd[1] = ps[1];
					pdd[2] = ps[2];
					ps += nVidImagePitch;
					pdd +=3;
				}
			}
			break;
		}
		case 2:	{
			for (int y = 0; y < nGameHeight; y++, pd += nPitch) {
				ps = pVidImage + (nGameHeight - 1 - y) * 2;
				pdd = pd;
				for (int x = 0; x < nGameWidth; x++) {
					*(short*)pdd = *(short*)ps;
					ps += nVidImagePitch;
					pdd += 2;
				}
			}
			break;
		}
	};
	
	DCFlushRange (texturemem, TEXTUREMEM_SIZE);	// update the texture memory
	GX_InvalidateTexAll ();
	draw_square (view);		// draw the quad
	GX_DrawDone ();
	VIDEO_SetNextFramebuffer (xfb[whichfb]);
	VIDEO_Flush ();
	copynow = GX_TRUE;
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

	vidBurnToSurface();
	return 0;
}

static int vidPaint(int bValidate)
{
	if (bValidate & 2) {
		vidBurnToSurface();
	}
	if (bValidate & 1) {
		// Win32 needs validate, Wii does not!
	}
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
