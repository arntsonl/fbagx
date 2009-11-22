#include "fileop.h"
#include "filebrowser.h"
#include "burner.h"
#include "menu.h"
#include <stdio.h>
#include <string.h>
#include <debug.h> // USB Gecko

int LoadBurnerRom(const char * fileName, const char * fileDir)
{
	// First we need the driver number of the filename
	char msg[256];
	snprintf(msg, 256, "Okay to load: %s out of %i found", fileName, nBurnDrvCount);
	if(WindowPrompt("Load Rom?", msg, "Ok", "Cancel"))
	{
		char* szName = NULL;
		char* pszRomName = NULL;
		bool bFound = false;
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
				DrvInitCallback();
				break;
			}
		}
		if(!bFound)
		{
			WindowPrompt("Rom Not Found", ":(", "Ok", "Cancel");
		}
	}
	return 0; // everything is ok
}