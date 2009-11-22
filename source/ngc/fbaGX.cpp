// FB Alpha - Emulator for MC68000/Z80 based arcade games
//            Refer to the "license.txt" file for more info

// Main module

// #define USE_SDL					// define if SDL is used
// #define DONT_DISPLAY_SPLASH		// Prevent Splash screen from being displayed
//#define APP_DEBUG_LOG			// log debug messages to zzBurnDebug.html

#include "burner.h"

#include <gccore.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ogcsys.h>
#include <unistd.h>
#include <wiiuse/wpad.h>
#include <fat.h>
#include <debug.h> // USB Gecko
#include <sys/iosupport.h>

#include "FreeTypeGX.h"
#include "video.h"
#include "audio.h"
#include "menu.h"
#include "input.h"
#include "filelist.h"
#include "fbaGX.h"

struct SSettings Settings;
int ExitRequested = 0;
int ShutdownRequested = 0;
int ResetRequested = 0;

void ExitApp()
{
	ShutoffRumble();
	StopGX();
	exit(0);
}

void
DefaultSettings()
{
	Settings.LoadMethod = METHOD_AUTO;
	Settings.SaveMethod = METHOD_AUTO;
	sprintf (Settings.Folder1,"libwiigui/first folder");
	sprintf (Settings.Folder2,"libwiigui/second folder");
	sprintf (Settings.Folder3,"libwiigui/third folder");
	Settings.AutoLoad = 1;
	Settings.AutoSave = 1;
}

void ShutdownCB()
{
	ShutdownRequested = 1;
}
void ResetCB()
{
	ResetRequested = 1;
}

void Emulate()
{
	// Initialize emulator settings?
	/*
	 * here
	 *
	 */
	MainMenu(MENU_SETTINGS); // make this game selection instead
	
	// Load game here, we've allocated everything in MainMenu and selected the file apparently
	
	while(1)
	{
		// FBA Main loop
		NGCReportButtons ();
		if(ResetRequested)
		{
			//S9xSoftReset (); // reset game
			ResetRequested = 0;
		}
		#ifdef HW_RVL
		if(ShutdownRequested)
			ExitApp();
		#endif
	}
}

int nAppVirtualFps = 6000;			// App fps * 100

static int AppInit()
{
	bCheatsAllowed = true;

	// Init the Burn library
	BurnLibInit();
	BurnDoGameListLocalisation();

	ComputeGammaLUT();

	if (VidSelect(nVidSelect)) {
		nVidSelect = 0;
		VidSelect(nVidSelect);
	}

	// Build the ROM information
	//CreateROMInfo(NULL);

	return 0;
}

static int AppExit()
{
	//DrvExit();						// Make sure any game driver is exitted
	//FreeROMInfo();
	//MediaExit();
	BurnLibExit();					// Exit the Burn library
	return 0;
}

void AppCleanup()
{
	AppExit();
}

/****************************************************************************
 * USB Gecko Debugging
 ***************************************************************************/
static bool gecko = false;
static mutex_t gecko_mutex = 0;

static ssize_t __out_write(struct _reent *r, int fd, const char *ptr, size_t len)
{
	u32 level;

	if (!ptr || len <= 0 || !gecko)
		return -1;

	LWP_MutexLock(gecko_mutex);
	level = IRQ_Disable();
	usb_sendbuffer(1, ptr, len);
	IRQ_Restore(level);
	LWP_MutexUnlock(gecko_mutex);
	return len;
}

const devoptab_t gecko_out = {
	"stdout",	// device name
	0,			// size of file structure
	NULL,		// device open
	NULL,		// device close
	__out_write,// device write
	NULL,		// device read
	NULL,		// device seek
	NULL,		// device fstat
	NULL,		// device stat
	NULL,		// device link
	NULL,		// device unlink
	NULL,		// device chdir
	NULL,		// device rename
	NULL,		// device mkdir
	0,			// dirStateSize
	NULL,		// device diropen_r
	NULL,		// device dirreset_r
	NULL,		// device dirnext_r
	NULL,		// device dirclose_r
	NULL		// device statvfs_r
};

void USBGeckoOutput()
{
	LWP_MutexInit(&gecko_mutex, false);
	gecko = usb_isgeckoalive(1);
	
	devoptab_list[STD_OUT] = &gecko_out;
	devoptab_list[STD_ERR] = &gecko_out;
}

// Main program entry point
int main(int argc, char *argv[])
{
	USBGeckoOutput();
	// Init USB Gecko First
	DEBUG_Init(GDBSTUB_DEVICE_USB,1);
	_break();
	InitVideo(); // Initialize video
	SetupPads(); // Initialize input
	InitAudio(); // Initialize audio
	fatInitDefault(); // Initialize file syste
	
	if (!(AppInit())) {							// Init the application
		// Some kinda warning?
	}

	InitFreeType((u8*)font_ttf, font_ttf_size); // Initialize font system
	InitGUIThreads(); // Initialize GUI
	
	// Wii Power/Reset buttons
	WPAD_SetPowerButtonCallback((WPADShutdownCallback)ShutdownCB);
	SYS_SetPowerCallback(ShutdownCB);
	SYS_SetResetCallback(ResetCB);
	
	DefaultSettings();
	
	MainMenu(MENU_SETTINGS);
	AppExit();									// Exit the application

	return 0;
}
