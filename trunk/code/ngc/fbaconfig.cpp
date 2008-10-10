/****************************************************************************
 * Final Burn Alpha Nintendo Wii/Gamecube Port
 *
 * some people
 * Cthulhu32 October 2008
 *
 * gbaconfig.h
 *
 * Configuration parameters are here for easy maintenance.
 ***************************************************************************/

#include <gccore.h>
#include <stdio.h>
#include <string.h>
#include "fbagx.h"

struct SGCSettings GCSettings;

void DefaultSettings ()
{
	/************** GameCube/Wii Settings *********************/
	GCSettings.LoadMethod = METHOD_AUTO; // Auto, SD, DVD, USB, Network (SMB)
	GCSettings.SaveMethod = METHOD_AUTO; // Auto, SD, Memory Card Slot A, Memory Card Slot B, USB, Network (SMB)
	sprintf (GCSettings.LoadFolder,"fbagx/roms"); // Path to game files
	sprintf (GCSettings.SaveFolder,"fbagx/saves"); // Path to save files
	sprintf (GCSettings.CheatFolder,"fbagx/cheats"); // Path to cheat files
	GCSettings.AutoLoad = 1;
	GCSettings.AutoSave = 1;

	GCSettings.VerifySaves = 0;

	// custom SMB settings
	strncpy (GCSettings.smbip, "", 15); // IP Address of share server
	strncpy (GCSettings.smbuser, "", 19); // Your share user
	strncpy (GCSettings.smbpwd, "", 19); // Your share user password
	strncpy (GCSettings.smbshare, "", 19); // Share name on server

	GCSettings.smbip[15] = 0;
	GCSettings.smbuser[19] = 0;
	GCSettings.smbpwd[19] = 0;
	GCSettings.smbshare[19] = 0;

	GCSettings.gcip[0] = 0;
	GCSettings.gwip[0] = 0;
	GCSettings.mask[0] = 0;
	GCSettings.smbsvid[0] = 0;
	GCSettings.smbgcid[0] = 0;

	GCSettings.NGCZoom = 0;	// zooming default off

	GCSettings.render = 2; // Unfiltered
	GCSettings.widescreen = 0; // no aspect ratio correction

	GCSettings.xshift = 0;	// video shift
	GCSettings.yshift = 0;

	/****************** FBAGX Settings ***********************/
}

