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

// Main program entry point
int main(int argc, char *argv[])
{
	InitVideo(); // Initialize video
	SetupPads(); // Initialize input
	InitAudio(); // Initialize audio
	fatInitDefault(); // Initialize file system
	
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
