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

int debugFillMenu(char ** menuArray){
	// We just fill up the menuArray, assuming the menu array has the space
	// So make sure we have Array[a lot][256];

	for(unsigned int z = 0; z < nBurnDrvCount; z++){
		BurnDrvGetZipName(&menuArray[z],z);	
	}
	return 0;

}

