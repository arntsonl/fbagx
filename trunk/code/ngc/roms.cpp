#include "roms.h"
#include "burn.h"

#include "fbagx.h"
#include "menudraw.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define BZIP_MAX 32

static char* szBzipName[BZIP_MAX] = { NULL, };					// Zip files to search through

struct RomFind { int nState; int nZip; int nPos; };				// State is non-zero if found. 1 = found totally okay.
static int nRomCount = 0; static int nTotalSize = 0;
static int nListCount = 0;
static int nCurrentZip = -1;									// Zip which is currently open

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

int checkRom(char * filename)
{
	for (unsigned int z = 0; z < nBurnDrvCount; z++) {
		nBurnDrvSelect = z;
		char * szName = NULL;
		BurnDrvGetZipName(&szName, 0); // this will return 1 if not found
		if (strncmp(szName,filename,strlen(szName))==0){ // not a memcpy :D
			char msg[256];
			sprintf(msg, "Found driver for %s, number %i/%i", szName, z, nBurnDrvCount);			
			WaitPrompt(msg);
			
			// Count the number of roms needed
			for (nRomCount = 0; ; nRomCount++) {
				if (BurnDrvGetRomInfo(NULL, nRomCount)) {
					break;
				}
			}
			if (nRomCount <= 0) {
				WaitPrompt((char *)"No roms found for nRomCount.");	
				return 1;
			}

			for (int i = 0; i < nRomCount; i++) {
				struct BurnRomInfo ri;
				memset(&ri, 0, sizeof(ri));
				if(BurnDrvGetRomInfo(&ri, i)) break;								// Get info about the rom
				char* romName = "Unknown";
				BurnDrvGetRomName(&romName,i, 0);
				if ( strlen(romName) == 0) continue;				
				char msg[256];
				sprintf(msg, "Rom name: %s %i/%i", romName, i, nRomCount);
				WaitPrompt(msg);			
			}
			return 1;		
		}	
	}
	char msg[256];
	sprintf(msg, "Could not find a driver for %s", filename);
	WaitPrompt(msg);
	return 0;
}
