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

#ifndef _FBAGX_H_
#define _FBAGX_H_

#include <gccore.h>

#define VERSIONNUM "001"
#define VERSIONSTR "FBA GX 001"

#define NOTSILENT 0
#define SILENT 1

enum {
        METHOD_AUTO,
        METHOD_SD,
        METHOD_USB,
        METHOD_DVD,
        METHOD_SMB,
        METHOD_MC_SLOTA,
        METHOD_MC_SLOTB
};

struct SGCSettings{
    int     AutoLoad;
    int     AutoSave;
    int     LoadMethod; // For ROMS: Auto, SD, DVD, USB, Network (SMB)
    int     SaveMethod; // For SRAM, Freeze, Prefs: Auto, SD, Memory Card Slot A, Memory Card Slot B, USB, SMB
    char    LoadFolder[200]; // Path to game files
    char    SaveFolder[200]; // Path to save files
    char    CheatFolder[200]; // Path to cheat files
    char    gcip[16];
    char    gwip[16];
    char    mask[16];
    char    smbip[16];
    char    smbuser[20];
    char    smbpwd[20];
    char    smbgcid[20];
    char    smbsvid[20];
    char    smbshare[20];
    int     NGCZoom; // 0 - off, 1 - on
    int     VerifySaves;
    int     render;         // 0 - original, 1 - filtered, 2 - unfiltered
    int     Justifier;
    int     widescreen;     // 0 - 4:3 aspect, 1 - 16:9 aspect
    int     xshift;         // video output shift
    int     yshift;
};


extern struct SGCSettings GCSettings;

#endif // _FBAGX_H_


