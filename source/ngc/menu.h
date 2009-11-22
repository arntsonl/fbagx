/****************************************************************************
 * libwiigui Template
 * Tantric 2009
 *
 * menu.h
 * Menu flow routines - handles all menu logic
 ***************************************************************************/

#ifndef _MENU_H_
#define _MENU_H_

#include <ogcsys.h>

void InitGUIThreads();
int WindowPrompt(const char*, const char*, const char*, const char*);
void MainMenu (int menuitem);

enum
{
	MENU_EXIT = -1,
	MENU_NONE,
	MENU_SETTINGS,
	MENU_SETTINGS_FILE,
	MENU_GAMESELECTION,
	MENU_BROWSE_DEVICE
};

#endif
