#include "roms.h"
#include "burn.h"

int AnalyzeRoms()
{
	// Go through the list of Zips
	for (unsigned int z = 0; z < nBurnDrvCount; z++) {
		nBurnDrvSelect = z;
		switch (0){//BzipOpen(true)) {
			case 0:
				//gameAv[z] = 3;
				break;
			case 2:
				//gameAv[z] = 1;
				break;
			case 1:
				//gameAv[z] = 0;
				break;	
		}
		//BzipClose();
	}

	// Check all the CRCs?

	// Redo their bZipOpen
	
	return 0;
}

