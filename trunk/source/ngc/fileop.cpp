#include "fileop.h"
#include "filebrowser.h"
#include "burner.h"
#include "menu.h"
#include <stdio.h>
#include <string.h>

bool LoadBurnerRom(const char * fileName, const char * fileDir)
{
	// First we need the driver number of the filename
	char msg[256];
	snprintf(msg, 256, "Okay to load: %s out of %i found (USB init? %i)", fileName, nBurnDrvCount, usb_isgeckoalive(1));
	bool bFound = false;
	if(WindowPrompt("Load Rom?", msg, "Ok", "Cancel"))
	{
		char* szName = NULL;
		char* pszRomName = NULL;
		
		for(int i = 0; i < nBurnDrvCount; i++)
		{
			nBurnDrvSelect = i;
			if(BurnDrvGetZipName(&szName, 0)) // returns a pointer to szName
			{
				break;
			}
			if(stricmp(szName,fileName) == 0)
			{
				snprintf(msg,256, "Rom Found at [%i]", i);
				WindowPrompt("Found Rom", msg, "Ok", "Cancel");
				bFound = true;
				break;
			}
		}
		
		if(!bFound)
		{
			WindowPrompt("Rom Not Found", ":(", "Ok", "Cancel");
		}
		else
		{
			AudSoundStop();						// Stop while the dialog is active or we're loading ROMs
			DrvInit(nBurnDrvSelect, true);		// Init the game driver (this should take the longest amount of time)
			if ( bDrvOkay )
			{
				// Takes a minute, so we want to add a progress bar fo sho
				WindowPrompt("Okay", "Rom Loaded Successfully", "Ok", "Cancel");
			}
			AudSoundPlay();						// Restart sound
		}
	}
	return bFound; // everything is ok
}