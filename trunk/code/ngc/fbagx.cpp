/****************************************************************************
 * Final Burn Alpha GX 0.1 Nintendo Wii/Gamecube Port
 * 
 * people
 * Cthulhu32 October 2008
 *
 * fbagx.h
 *
 * This file is very close to Snes9xgx, and controls starts & ends!
 ****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <ogcsys.h>
#include <unistd.h>
#include <wiiuse/wpad.h>
#include <sdcard/card_cmn.h>
#include <sdcard/wiisd_io.h>
#include <sdcard/card_io.h>
#include <fat.h>

#include "fbaconfig.h"

#ifdef WII_DVD
extern "C" {
#include <di/di.h>
}
#endif

#include "video.h"
#include "fbagx.h"

// more stuff will go here
void emulate(){
    // setup all inits for FBA

    while(1)
    {
        // do the FBA main loop
        // do some buttons
        // check for config request (maybe throw this in a listener thread)
    }
}

/****************************************************************************
 * MAIN
 * 
 * Steps to FBAgx Emulation:
 *     1. Initialise GC Video
 *     2. Initialise libfreetype (Nice to read something)
 *     3. Setup standard FBA settings
 *     4. Allocate FBA memory
 *     5. more stuff?
 *     6. Set pixel format to RGB565 for GL Rendering
 *     7. Initialise FBA/GC Sound System
 *     8. Initialise FBA Graphics Subsystem
 *     9. Let's party like its Snes9xGX
 * 
 ****************************************************************************/
int main(){

#ifdef WII_DVD
    DI_Init();
#endif

    unsigned int saveflags;
    int selectedMenu = -1;

    // Initialise the video
    InitGCVideo();

    // Controllers
    WPAD_Init();
    WPAD_SetDataFormat(WPAD_CHAN_ALL, WPAD_FMT_BTNS_ACC_IR);
    WPAD_SetVRes(WPAD_CHAN_ALL, 640, 480);

    PAD_Init();
    // Audio init
    AUDIO_Init(NULL);

    // Set Default Settings
    DefaultSettings();
/*
    FBAUnmapAllControls();
    SetDefaultButtonMap();

    // Allocate FBAGX memory
    if ( !Memory.Init() )
       while (1);

    // Allocate APU
    if ( !FBAInitAPU())
        while (1);

    FBASetRenderPixelFormat(RGB565);

    FBAInitSound(5, True, 1024);

    setGFX();
    if ( !FBAGraphicsInit())
        while(1);

    // Initialize libFAT for SD and USB
    fatInit( 8, false);

    // Check if DVD drive belongs to the Wii
    SetDVDDriveType();

    if(!LoadPrefs())
    {
        WaitPrompt((char*) "Preferences reset - check settings!");
        selectedMenu = 1;
    }
*/
    // Load our user selected prompt
    emulate();

    // leaving, not the best way to leave the emulator
    return 0;
}

