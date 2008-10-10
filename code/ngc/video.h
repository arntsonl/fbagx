/****************************************************************************
 * Final Burn Alpha GX 0.1 Nintendo Wii/Gamecube Port
 * 
 * people
 * Cthulhu32 October 2008
 *
 * video.h
 *
 * Video routines
 ****************************************************************************/

#ifndef _VIDEOH_
#define _VIDEOH_

#include <ogcsys.h>

void InitGCVideo();
void ResetVideo_Emu();
void ResetVideo_Menu();
void setGFX();
void update_video(int width, int height);
void clearscreen (int colour = COLOR_BLACK);
void showscreen();
void zoom (float speed);
void zoom_reset();

extern bool progressive;

#endif
