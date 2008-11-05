// Burner Game Input
#include "burner.h"
/*
// Player Default Controls
int nPlayerDefaultControls[4] = {0, 1, 2, 3};
char szPlayerDefaultIni[4][MAX_PATH] = { (""), (""), (""), ("") };

// Mapping of PC inputs to game inputs
struct GameInp* GameInp = NULL;
unsigned int nGameInpCount = 0;
unsigned int nMacroCount = 0;
unsigned int nMaxMacro = 0;

int nAnalogSpeed;

int nFireButtons = 0;

bool bStreetFighterLayout = false;
bool bLeftAltkeyMapped = false;

// ---------------------------------------------------------------------------

// Check if the left alt (menu) key is mapped
void GameInpCheckLeftAlt()
{
	struct GameInp* pgi;
	unsigned int i;

	bLeftAltkeyMapped = false;

	for (i = 0, pgi = GameInp; i < (nGameInpCount + nMacroCount); i++, pgi++) {

		if (bLeftAltkeyMapped) {
			break;
		}

		switch (pgi->nInput) {
			case GIT_SWITCH:
				if (pgi->Input.Switch.nCode == FBK_LALT) {
					bLeftAltkeyMapped = true;
				}
				break;
			case GIT_MACRO_AUTO:
			case GIT_MACRO_CUSTOM:
				if (pgi->Macro.nMode) {
					if (pgi->Macro.Switch.nCode == FBK_LALT) {
						bLeftAltkeyMapped = true;
					}
				}
				break;

			default:
				continue;
		}
	}
}

// Check if the sytem mouse is mapped and set the cooperative level apropriately
void GameInpCheckMouse()
{
	bool bMouseMapped = false;
	struct GameInp* pgi;
	unsigned int i;

	for (i = 0, pgi = GameInp; i < (nGameInpCount + nMacroCount); i++, pgi++) {

		if (bMouseMapped) {
			break;
		}

		switch (pgi->nInput) {
			case GIT_SWITCH:
				if ((pgi->Input.Switch.nCode & 0xFF00) == 0x8000) {
					bMouseMapped = true;
				}
				break;
			case GIT_MOUSEAXIS:
				if (pgi->Input.MouseAxis.nMouse == 0) {
					bMouseMapped = true;
				}
				break;
			case GIT_MACRO_AUTO:
			case GIT_MACRO_CUSTOM:
				if (pgi->Macro.nMode) {
					if ((pgi->Macro.Switch.nCode & 0xFF00) == 0x8000) {
						bMouseMapped = true;
					}
				}
				break;

			default:
				continue;
		}
	}

	if (bDrvOkay) {
		if (!bRunPause) {
			InputSetCooperativeLevel(bMouseMapped, bAlwaysProcessKeyboardInput);
		} else {
			InputSetCooperativeLevel(false, bAlwaysProcessKeyboardInput);
		}
	} else {
		InputSetCooperativeLevel(false, false);
	}
}

// ---------------------------------------------------------------------------

int GameInpBlank(int bDipSwitch)
{
	unsigned int i = 0;
	struct GameInp* pgi = NULL;

	// Reset all inputs to undefined (even dip switches, if bDipSwitch==1)
	if (GameInp == NULL) {
		return 1;
	}

	// Get the targets in the library for the Input Values
	for (i = 0, pgi = GameInp; i < nGameInpCount; i++, pgi++) {
		struct BurnInputInfo bii;
		memset(&bii, 0, sizeof(bii));
		BurnDrvGetInputInfo(&bii, i);
		if (bDipSwitch == 0 && (bii.nType & BIT_GROUP_CONSTANT)) {		// Don't blank the dip switches
			continue;
		}

		memset(pgi, 0, sizeof(*pgi));									// Clear input

		pgi->nType = bii.nType;											// store input type
		pgi->Input.pVal = bii.pVal;										// store input pointer to value

		if (bii.nType & BIT_GROUP_CONSTANT) {							// Further initialisation for constants/DIPs
			pgi->nInput = GIT_CONSTANT;
			pgi->Input.Constant.nConst = *bii.pVal;
		}
	}

	for (i = 0; i < nMacroCount; i++, pgi++) {
		pgi->Macro.nMode = 0;
		if (pgi->nInput == GIT_MACRO_CUSTOM) {
			pgi->nInput = 0;
		}
	}

	bLeftAltkeyMapped = false;

	return 0;
}

static void GameInpInitMacros()
{
	struct GameInp* pgi;
	struct BurnInputInfo bii;

	int nPunchx3[4] = {0, 0, 0, 0};
	int nPunchInputs[4][3];
	int nKickx3[4] = {0, 0, 0, 0};
	int nKickInputs[4][3];

	int nNeogeoButtons[4][4];

	bStreetFighterLayout = false;
	nMacroCount = 0;

	nFireButtons = 0;

	for (unsigned int i = 0; i < nGameInpCount; i++) {
		bii.szName = NULL;
		BurnDrvGetInputInfo(&bii, i);
		if (bii.szName == NULL) {
			bii.szName = "";
		}
		if (bii.szName[0] == 'P' && bii.szName[1] >= '1' && bii.szName[1] <= '4') {
			int nPlayer = bii.szName[1] - '1';

			if (nPlayer == 0) {
				if (strncmp(" fire", bii.szInfo + 2, 5) == 0) {
					nFireButtons++;
				}
			}

			if (_stricmp(" Weak Punch", bii.szName + 2) == 0) {
				nPunchx3[nPlayer] |= 1;
				nPunchInputs[nPlayer][0] = i;
			}
			if (_stricmp(" Medium Punch", bii.szName + 2) == 0) {
				nPunchx3[nPlayer] |= 2;
				nPunchInputs[nPlayer][1] = i;
			}
			if (_stricmp(" Strong Punch", bii.szName + 2) == 0) {
				nPunchx3[nPlayer] |= 4;
				nPunchInputs[nPlayer][2] = i;
			}
			if (_stricmp(" Weak Kick", bii.szName + 2) == 0) {
				nKickx3[nPlayer] |= 1;
				nKickInputs[nPlayer][0] = i;
			}
			if (_stricmp(" Medium Kick", bii.szName + 2) == 0) {
				nKickx3[nPlayer] |= 2;
				nKickInputs[nPlayer][1] = i;
			}
			if (_stricmp(" Strong Kick", bii.szName + 2) == 0) {
				nKickx3[nPlayer] |= 4;
				nKickInputs[nPlayer][2] = i;
			}

			if (_stricmp(" Button A", bii.szName + 2) == 0) {
				nNeogeoButtons[nPlayer][0] = i;
			}
			if (_stricmp(" Button B", bii.szName + 2) == 0) {
				nNeogeoButtons[nPlayer][1] = i;
			}
			if (_stricmp(" Button C", bii.szName + 2) == 0) {
				nNeogeoButtons[nPlayer][2] = i;
			}
			if (_stricmp(" Button D", bii.szName + 2) == 0) {
				nNeogeoButtons[nPlayer][3] = i;
			}
		}
	}

	pgi = GameInp + nGameInpCount;

	for (int nPlayer = 0; nPlayer < nMaxPlayers; nPlayer++) {
		if (nPunchx3[nPlayer] == 7) {		// Create a 3x punch macro
			pgi->nInput = GIT_MACRO_AUTO;
			pgi->nType = BIT_DIGITAL;
			pgi->Macro.nMode = 0;

			sprintf(pgi->Macro.szName, "P%i 3× Punch", nPlayer + 1);
			for (int j = 0; j < 3; j++) {
				BurnDrvGetInputInfo(&bii, nPunchInputs[nPlayer][j]);
				pgi->Macro.pVal[j] = bii.pVal;
				pgi->Macro.nVal[j] = 1;
			}

			nMacroCount++;
			pgi++;
		}

		if (nKickx3[nPlayer] == 7) {		// Create a 3x kick macro
			pgi->nInput = GIT_MACRO_AUTO;
			pgi->nType = BIT_DIGITAL;
			pgi->Macro.nMode = 0;

			sprintf(pgi->Macro.szName, "P%i 3× Kick", nPlayer + 1);
			for (int j = 0; j < 3; j++) {
				BurnDrvGetInputInfo(&bii, nKickInputs[nPlayer][j]);
				pgi->Macro.pVal[j] = bii.pVal;
				pgi->Macro.nVal[j] = 1;
			}

			nMacroCount++;
			pgi++;
		}

		if (nFireButtons == 4 && (BurnDrvGetHardwareCode() & HARDWARE_PUBLIC_MASK) == HARDWARE_SNK_NEOGEO) {

			// A + B + C macro
			pgi->nInput = GIT_MACRO_AUTO;
			pgi->nType = BIT_DIGITAL;
			pgi->Macro.nMode = 0;

			sprintf(pgi->Macro.szName, "P%i Button ABC", nPlayer + 1);
			BurnDrvGetInputInfo(&bii, nNeogeoButtons[nPlayer][0]);
			pgi->Macro.pVal[0] = bii.pVal;
			pgi->Macro.nVal[0] = 1;
			BurnDrvGetInputInfo(&bii, nNeogeoButtons[nPlayer][1]);
			pgi->Macro.pVal[1] = bii.pVal;
			pgi->Macro.nVal[1] = 1;
			BurnDrvGetInputInfo(&bii, nNeogeoButtons[nPlayer][2]);
			pgi->Macro.pVal[2] = bii.pVal;
			pgi->Macro.nVal[2] = 1;

			nMacroCount++;
			pgi++;

			// B + C + D macro
			pgi->nInput = GIT_MACRO_AUTO;
			pgi->nType = BIT_DIGITAL;
			pgi->Macro.nMode = 0;

			sprintf(pgi->Macro.szName, "P%i Button BCD", nPlayer + 1);
			BurnDrvGetInputInfo(&bii, nNeogeoButtons[nPlayer][1]);
			pgi->Macro.pVal[0] = bii.pVal;
			pgi->Macro.nVal[0] = 1;
			BurnDrvGetInputInfo(&bii, nNeogeoButtons[nPlayer][2]);
			pgi->Macro.pVal[1] = bii.pVal;
			pgi->Macro.nVal[1] = 1;
			BurnDrvGetInputInfo(&bii, nNeogeoButtons[nPlayer][3]);
			pgi->Macro.pVal[2] = bii.pVal;
			pgi->Macro.nVal[2] = 1;

			nMacroCount++;
			pgi++;

			// A + B + C + D macro
			pgi->nInput = GIT_MACRO_AUTO;
			pgi->nType = BIT_DIGITAL;
			pgi->Macro.nMode = 0;

			sprintf(pgi->Macro.szName, "P%i Button ABCD", nPlayer + 1);
			BurnDrvGetInputInfo(&bii, nNeogeoButtons[nPlayer][0]);
			pgi->Macro.pVal[0] = bii.pVal;
			pgi->Macro.nVal[0] = 1;
			BurnDrvGetInputInfo(&bii, nNeogeoButtons[nPlayer][1]);
			pgi->Macro.pVal[1] = bii.pVal;
			pgi->Macro.nVal[1] = 1;
			BurnDrvGetInputInfo(&bii, nNeogeoButtons[nPlayer][2]);
			pgi->Macro.pVal[2] = bii.pVal;
			pgi->Macro.nVal[2] = 1;
			BurnDrvGetInputInfo(&bii, nNeogeoButtons[nPlayer][3]);
			pgi->Macro.pVal[3] = bii.pVal;
			pgi->Macro.nVal[3] = 1;

			nMacroCount++;
			pgi++;
		}
	}

	if ((nPunchx3[0] == 7) && (nKickx3[0] == 7)) {
		bStreetFighterLayout = true;
	}
	if (nFireButtons >= 5 && (BurnDrvGetHardwareCode() & HARDWARE_PUBLIC_MASK) == HARDWARE_CAPCOM_CPS2) {
		bStreetFighterLayout = true;
	}
}

int GameInpInit()
{
	int nRet = 0;
	// Count the number of inputs
	nGameInpCount = 0;
	nMacroCount = 0;
	nMaxMacro = nMaxPlayers * 8;

	for (unsigned int i = 0; i < 0x1000; i++) {
		nRet = BurnDrvGetInputInfo(NULL,i);
		if (nRet) {														// end of input list
			nGameInpCount = i;
			break;
		}
	}

	// Allocate space for all the inputs
	int nSize = (nGameInpCount + nMaxMacro) * sizeof(struct GameInp);
	GameInp = (struct GameInp*)malloc(nSize);
	if (GameInp == NULL) {
		return 1;
	}
	memset(GameInp, 0, nSize);

	GameInpBlank(1);

	InpDIPSWResetDIPs();

	GameInpInitMacros();

	nAnalogSpeed = 0x0100;

	return 0;
}

int GameInpExit()
{
	free(GameInp);
	GameInp = NULL;

	nGameInpCount = 0;
	nMacroCount = 0;

	nFireButtons = 0;

	bStreetFighterLayout = false;
	bLeftAltkeyMapped = false;

	return 0;
}

// ---------------------------------------------------------------------------
// Convert a string from a config file to an input

static char* SliderInfo(struct GameInp* pgi, char* s)
{
	char* szRet = NULL;
	pgi->Input.Slider.nSliderSpeed = 0x700;				// defaults
	pgi->Input.Slider.nSliderCenter = 0;
	pgi->Input.Slider.nSliderValue = 0x8000;

	szRet = LabelCheck(s, ("speed"));
	s = szRet;
	if (s == NULL) {
		return s;
	}
	pgi->Input.Slider.nSliderSpeed = (short)cstol(s, &szRet, 0);
	s = szRet;
	if (s==NULL) {
		return s;
	}
	szRet = LabelCheck(s, ("center"));
	s = szRet;
	if (s == NULL) {
		return s;
	}
	pgi->Input.Slider.nSliderCenter = (short)cstol(s, &szRet, 0);
	s = szRet;
	if (s == NULL) {
		return s;
	}

	return szRet;
}

static int StringToJoyAxis(struct GameInp* pgi, char* s)
{
	char* szRet = s;

	pgi->Input.JoyAxis.nJoy = (unsigned char)cstol(s, &szRet, 0);
	if (szRet == NULL) {
		return 1;
	}
	s = szRet;
	pgi->Input.JoyAxis.nAxis = (unsigned char)cstol(s, &szRet, 0);
	if (szRet == NULL) {
		return 1;
	}

	return 0;
}

static int StringToMouseAxis(struct GameInp* pgi, char* s)
{
	char* szRet = s;

	pgi->Input.MouseAxis.nAxis = (unsigned char)cstol(s, &szRet, 0);
	if (szRet == NULL) {
		return 1;
	}

	return 0;
}

static int StringToMacro(struct GameInp* pgi, char* s)
{
	char* szRet = NULL;

	szRet = LabelCheck(s, ("switch"));
	if (szRet) {
		s = szRet;
		pgi->Macro.nMode = 0x01;
		pgi->Macro.Switch.nCode = (unsigned short)cstol(s, &szRet, 0);
		return 0;
	}

	return 1;
}

static int StringToInp(struct GameInp* pgi, char* s)
{
	char* szRet = NULL;

	SKIP_WS(s);											// skip whitespace
	szRet = LabelCheck(s, ("undefined"));
	if (szRet) {
		pgi->nInput = 0;
		return 0;
	}

	szRet = LabelCheck(s, ("constant"));
	if (szRet) {
		pgi->nInput = GIT_CONSTANT;
		s = szRet;
		pgi->Input.Constant.nConst=(unsigned char)cstol(s, &szRet, 0);
		*(pgi->Input.pVal) = pgi->Input.Constant.nConst;
		return 0;
	}

	szRet = LabelCheck(s, ("switch"));
	if (szRet) {
		pgi->nInput = GIT_SWITCH;
		s = szRet;
		pgi->Input.Switch.nCode = (unsigned short)cstol(s, &szRet, 0);
		return 0;
	}

	// Analog using mouse axis:
	szRet = LabelCheck(s, ("mouseaxis"));
	if (szRet) {
		pgi->nInput = GIT_MOUSEAXIS;
		return StringToMouseAxis(pgi, szRet);
	}
	// Analog using joystick axis:
	szRet = LabelCheck(s, ("joyaxis-neg"));
	if (szRet) {
		pgi->nInput = GIT_JOYAXIS_NEG;
		return StringToJoyAxis(pgi, szRet);
	}
	szRet = LabelCheck(s, ("joyaxis-pos"));
	if (szRet) {
		pgi->nInput = GIT_JOYAXIS_POS;
		return StringToJoyAxis(pgi, szRet);
	}
	szRet = LabelCheck(s, ("joyaxis"));
	if (szRet) {
		pgi->nInput = GIT_JOYAXIS_FULL;
		return StringToJoyAxis(pgi, szRet);
	}

	// Analog using keyboard slider
	szRet = LabelCheck(s, ("slider"));
	if (szRet) {
		s = szRet;
		pgi->nInput = GIT_KEYSLIDER;
		pgi->Input.Slider.SliderAxis.nSlider[0] = 0;	// defaults
		pgi->Input.Slider.SliderAxis.nSlider[1] = 0;	//

		pgi->Input.Slider.SliderAxis.nSlider[0] = (unsigned short)cstol(s, &szRet, 0);
		s = szRet;
		if (s == NULL) {
			return 1;
		}
		pgi->Input.Slider.SliderAxis.nSlider[1] = (unsigned short)cstol(s, &szRet, 0);
		s = szRet;
		if (s == NULL) {
			return 1;
		}
		szRet = SliderInfo(pgi, s);
		s = szRet;
		if (s == NULL) {								// Get remaining slider info
			return 1;
		}
		return 0;
	}

	// Analog using joystick slider
	szRet = LabelCheck(s, ("joyslider"));
	if (szRet) {
		s = szRet;
		pgi->nInput = GIT_JOYSLIDER;
		pgi->Input.Slider.JoyAxis.nJoy = 0;				// defaults
		pgi->Input.Slider.JoyAxis.nAxis = 0;			//

		pgi->Input.Slider.JoyAxis.nJoy = (unsigned char)cstol(s, &szRet, 0);
		s = szRet;
		if (s == NULL) {
			return 1;
		}
		pgi->Input.Slider.JoyAxis.nAxis = (unsigned char)cstol(s, &szRet, 0);
		s = szRet;
		if (s == NULL) {
			return 1;
		}
		szRet = SliderInfo(pgi, s);						// Get remaining slider info
		s = szRet;
		if (s == NULL) {
			return 1;
		}
		return 0;
	}

	return 1;
}

// ---------------------------------------------------------------------------
// Convert an input to a string for config files

static char* InpToString(struct GameInp* pgi)
{
	static char szString[80];

	if (pgi->nInput == 0) {
		return ("undefined");
	}
	if (pgi->nInput == GIT_CONSTANT) {
		_stprintf(szString, ("constant 0x%.2X"), pgi->Input.Constant.nConst);
		return szString;
	}
	if (pgi->nInput == GIT_SWITCH) {
		_stprintf(szString, ("switch 0x%.2X"), pgi->Input.Switch.nCode);
		return szString;
	}
	if (pgi->nInput == GIT_KEYSLIDER) {
		_stprintf(szString, ("slider 0x%.2x 0x%.2x speed 0x%x center %d"), pgi->Input.Slider.SliderAxis.nSlider[0], pgi->Input.Slider.SliderAxis.nSlider[1], pgi->Input.Slider.nSliderSpeed, pgi->Input.Slider.nSliderCenter);
		return szString;
	}
	if (pgi->nInput == GIT_JOYSLIDER) {
		_stprintf(szString, ("joyslider %d %d speed 0x%x center %d"), pgi->Input.Slider.JoyAxis.nJoy, pgi->Input.Slider.JoyAxis.nAxis, pgi->Input.Slider.nSliderSpeed, pgi->Input.Slider.nSliderCenter);
		return szString;
	}
	if (pgi->nInput == GIT_MOUSEAXIS) {
		_stprintf(szString, ("mouseaxis %d"), pgi->Input.MouseAxis.nAxis);
		return szString;
	}
	if (pgi->nInput == GIT_JOYAXIS_FULL) {
		_stprintf(szString, ("joyaxis %d %d"), pgi->Input.JoyAxis.nJoy, pgi->Input.JoyAxis.nAxis);
		return szString;
	}
	if (pgi->nInput == GIT_JOYAXIS_NEG) {
		_stprintf(szString, ("joyaxis-neg %d %d"), pgi->Input.JoyAxis.nJoy, pgi->Input.JoyAxis.nAxis);
		return szString;
	}
	if (pgi->nInput == GIT_JOYAXIS_POS) {
		_stprintf(szString, ("joyaxis-pos %d %d"), pgi->Input.JoyAxis.nJoy, pgi->Input.JoyAxis.nAxis);
		return szString;
	}

	return ("unknown");
}

static char* InpMacroToString(struct GameInp* pgi)
{
	static char szString[256];

	if (pgi->nInput == GIT_MACRO_AUTO) {
		if (pgi->Macro.nMode) {
			_stprintf(szString, ("switch 0x%.2X"), pgi->Macro.Switch.nCode);
			return szString;
		}
	}

	if (pgi->nInput == GIT_MACRO_CUSTOM) {
		struct BurnInputInfo bii;

		if (pgi->Macro.nMode) {
			_stprintf(szString, ("switch 0x%.2X"), pgi->Macro.Switch.nCode);
		} else {
			_stprintf(szString, ("undefined"));
		}

		for (int i = 0; i < 4; i++) {
			if (pgi->Macro.pVal[i]) {
				BurnDrvGetInputInfo(&bii, pgi->Macro.nInput[i]);
				_stprintf(szString + cslen(szString), (" \"%hs\" 0x%02X"), bii.szName, pgi->Macro.nVal[i]);
			}
		}

		return szString;
	}

	return ("undefined");
}

char* InputCodeDesc(int c)
{
	static char szString[64];
	char* szName = ("");

	// Mouse
	if (c >= 0x8000) {
		int nMouse = (c >> 8) & 0x3F;
		int nCode = c & 0xFF;
		if (nCode >= 0x80) {
			_stprintf(szString, ("Mouse %d Button %d"), nMouse, nCode & 0x7F);
			return szString;
		}
		if (nCode < 0x06) {
			char szAxis[3][3] = { ("X"), ("Y"), ("Z") };
			char szDir[6][16] = { ("negative"), ("positive"), ("Left"), ("Right"), ("Up"), ("Down") };
			if (nCode < 4) {
				_stprintf(szString, ("Mouse %d %s (%s %s)"), nMouse, szDir[nCode + 2], szAxis[nCode >> 1], szDir[nCode & 1]);
			} else {
				_stprintf(szString, ("Mouse %d %s %s"), nMouse, szAxis[nCode >> 1], szDir[nCode & 1]);
			}
			return szString;
		}
	}

	// Joystick
	if (c >= 0x4000 && c < 0x8000) {
		int nJoy = (c >> 8) & 0x3F;
		int nCode = c & 0xFF;
		if (nCode >= 0x80) {
			_stprintf(szString, ("Joy %d Button %d"), nJoy, nCode & 0x7F);
			return szString;
		}
		if (nCode < 0x10) {
			char szAxis[8][3] = { ("X"), ("Y"), ("Z"), ("rX"), ("rY"), ("rZ"), ("s0"), ("s1") };
			char szDir[6][16] = { ("negative"), ("positive"), ("Left"), ("Right"), ("Up"), ("Down") };
			if (nCode < 4) {
				_stprintf(szString, ("Joy %d %s (%s %s)"), nJoy, szDir[nCode + 2], szAxis[nCode >> 1], szDir[nCode & 1]);
			} else {
				_stprintf(szString, ("Joy %d %s %s"), nJoy, szAxis[nCode >> 1], szDir[nCode & 1]);
			}
			return szString;
		}
		if (nCode < 0x20) {
			char szDir[4][16] = { ("Left"), ("Right"), ("Up"), ("Down") };
			_stprintf(szString, ("Joy %d POV-hat %d %s"), nJoy, (nCode & 0x0F) >> 2, szDir[nCode & 3]);
			return szString;
		}
	}

	for (int i = 0; KeyNames[i].nCode; i++) {
		if (c == KeyNames[i].nCode) {
			if (KeyNames[i].szName) {
				szName = KeyNames[i].szName;
			}
			break;
		}
	}

	if (szName[0]) {
		_stprintf(szString, ("%s"), szName);
	} else {
		_stprintf(szString, ("code 0x%.2X"), c);
	}

	return szString;
}

char* InpToDesc(struct GameInp* pgi)
{
	static char szInputName[64] = ("");

	if (pgi->nInput == 0) {
		return ("");
	}
	if (pgi->nInput == GIT_CONSTANT) {
		if (pgi->nType & BIT_GROUP_CONSTANT) {
			for (int i = 0; i < 8; i++) {
				szInputName[7 - i] = pgi->Input.Constant.nConst & (1 << i) ? ('1') : ('0');
			}
			szInputName[8] = 0;

			return szInputName;
		}

		if (pgi->Input.Constant.nConst == 0) {
			return ("-");
		}
	}
	if (pgi->nInput == GIT_SWITCH) {
		return InputCodeDesc(pgi->Input.Switch.nCode);
	}
	if (pgi->nInput == GIT_MOUSEAXIS) {
		char nAxis = ('?');
		switch (pgi->Input.MouseAxis.nAxis) {
			case 0:
				nAxis = ('X');
				break;
			case 1:
				nAxis = ('Y');
				break;
			case 2:
				nAxis = ('Z');
				break;
		}
		_stprintf(szInputName, ("Mouse %i %c axis"), pgi->Input.MouseAxis.nMouse, nAxis);
		return szInputName;
	}
	if (pgi->nInput & GIT_GROUP_JOYSTICK) {
		char szAxis[8][3] = { ("X"), ("Y"), ("Z"), ("rX"), ("rY"), ("rZ"), ("s0"), ("s1") };
		char szRange[4][16] = { ("unknown"), ("full"), ("negative"), ("positive") };
		int nRange = 0;
		switch (pgi->nInput) {
			case GIT_JOYAXIS_FULL:
				nRange = 1;
				break;
			case GIT_JOYAXIS_NEG:
				nRange = 2;
				break;
			case GIT_JOYAXIS_POS:
				nRange = 3;
				break;
		}

		_stprintf(szInputName, ("Joy %d %s axis (%s range)"), pgi->Input.JoyAxis.nJoy, szAxis[pgi->Input.JoyAxis.nAxis], szRange[nRange]);
		return szInputName;
	}

	return InpToString(pgi);							// Just do the rest as they are in the config file
}

char* InpMacroToDesc(struct GameInp* pgi)
{
	if (pgi->nInput & GIT_GROUP_MACRO) {
		if (pgi->Macro.nMode) {
			return InputCodeDesc(pgi->Macro.Switch.nCode);
		}
	}

	return ("");
}

// ---------------------------------------------------------------------------

// Find the input number by info
static unsigned int InputInfoToNum(char* szName)
{
	for (unsigned int i = 0; i < nGameInpCount; i++) {
		struct BurnInputInfo bii;
		BurnDrvGetInputInfo(&bii, i);
		if (bii.pVal == NULL) {
			continue;
		}

		if (csicmp(szName, ANSITochar(bii.szInfo, NULL, 0)) == 0) {
			return i;
		}
	}
	return ~0U;
}

// Find the input number by name
static unsigned int InputNameToNum(char* szName)
{
	for (unsigned int i = 0; i < nGameInpCount; i++) {
		struct BurnInputInfo bii;
		BurnDrvGetInputInfo(&bii, i);
		if (bii.pVal == NULL) {
			continue;
		}

		if (csicmp(szName, ANSITochar(bii.szName, NULL, 0)) == 0) {
			return i;
		}
	}
	return ~0U;
}

static char* InputNumToName(unsigned int i)
{
	struct BurnInputInfo bii;
	bii.szName = NULL;
	BurnDrvGetInputInfo(&bii, i);
	if (bii.szName == NULL) {
		return ("unknown");
	}
	return ANSITochar(bii.szName, NULL, 0);
}

static unsigned int MacroNameToNum(char* szName)
{
	struct GameInp* pgi = GameInp + nGameInpCount;
	for (unsigned int i = 0; i < nMacroCount; i++, pgi++) {
		if (pgi->nInput & GIT_GROUP_MACRO) {
			if (csicmp(szName, ANSITochar(pgi->Macro.szName, NULL, 0)) == 0) {
				return i;
			}
		}
	}
	return ~0U;
}

// ---------------------------------------------------------------------------

static int GameInpAutoOne(struct GameInp* pgi, char* szi)
{
	for (int i = 0; i < nMaxPlayers; i++) {
		int nSlide = nPlayerDefaultControls[i] >> 4;
		switch (nPlayerDefaultControls[i] & 0x0F) {
			case 0:										// Keyboard
				GamcAnalogKey(pgi, szi, i, nSlide);
				GamcPlayer(pgi, szi, i, -1);
				GamcMisc(pgi, szi, i);
				break;
			case 1:										// Joystick 1
				GamcAnalogJoy(pgi, szi, i, 0, nSlide);
				GamcPlayer(pgi, szi, i, 0);
				GamcMisc(pgi, szi, i);
				break;
			case 2:										// Joystick 2
				GamcAnalogJoy(pgi, szi, i, 1, nSlide);
				GamcPlayer(pgi, szi, i, 1);
				GamcMisc(pgi, szi, i);
				break;
			case 3:										// Joystick 3
				GamcAnalogJoy(pgi, szi, i, 2, nSlide);
				GamcPlayer(pgi, szi, i, 2);
				GamcMisc(pgi, szi, i);
				break;
			case 4:										// X-Arcade left side
				GamcMisc(pgi, szi, i);
				GamcPlayerHotRod(pgi, szi, i, 0x10, nSlide);
				break;
			case 5:										// X-Arcade right side
				GamcMisc(pgi, szi, i);
				GamcPlayerHotRod(pgi, szi, i, 0x11, nSlide);
				break;
			case 6:										// Hot Rod left side
				GamcMisc(pgi, szi, i);
				GamcPlayerHotRod(pgi, szi, i, 0x00, nSlide);
				break;
			case 7:										// Hot Rod right side
				GamcMisc(pgi, szi, i);
				GamcPlayerHotRod(pgi, szi, i, 0x01, nSlide);
				break;
			default:
				GamcMisc(pgi, szi, i);
		}
	}

	return 0;
}

static int AddCustomMacro(char* szValue, bool bOverWrite)
{
	char* szQuote = NULL;
	char* szEnd = NULL;

	if (QuoteRead(&szQuote, &szEnd, szValue)) {
		return 1;
	}

	int nMode = -1;
	int nInput = -1;
	bool bCreateNew = false;
	struct BurnInputInfo bii;

	for (unsigned int j = nGameInpCount; j < nGameInpCount + nMacroCount; j++) {
		if (GameInp[j].nInput == GIT_MACRO_CUSTOM) {
			if (LabelCheck(szQuote, ANSITochar(GameInp[j].Macro.szName, NULL, 0))) {
				nInput = j;
				break;
			}
		}
	}

	if (nInput == -1) {
		if (nMacroCount + 1 == nMaxMacro) {
			return 1;
		}
		nInput = nGameInpCount + nMacroCount;
		bCreateNew = true;
	}

	cscpy(szQuote, ANSITochar(GameInp[nInput].Macro.szName, NULL, 0));

	if ((szValue = LabelCheck(szEnd, ("undefined"))) != NULL) {
		nMode = 0;
	} else {
		if ((szValue = LabelCheck(szEnd, ("switch"))) != NULL) {

			if (bOverWrite || GameInp[nInput].Macro.nMode == 0) {
				GameInp[nInput].Macro.Switch.nCode = (unsigned short)cstol(szValue, &szValue, 0);
			}

			nMode = 1;
		}
	}

	if (nMode >= 0) {
		int nFound = 0;

		for (int i = 0; i < 4; i++) {
			GameInp[nInput].Macro.pVal[i] = NULL;
			GameInp[nInput].Macro.nVal[i] = 0;
			GameInp[nInput].Macro.nInput[i] = 0;

			if (szValue == NULL) {
				break;
			}

			if (QuoteRead(&szQuote, &szEnd, szValue)) {
				break;
			}

			for (unsigned int j = 0; j < nGameInpCount; j++) {
				bii.szName = NULL;
				BurnDrvGetInputInfo(&bii, j);
				if (bii.pVal == NULL) {
					continue;
				}

				char* szString = LabelCheck(szQuote, ANSITochar(bii.szName, NULL, 0));
				if (szString && szEnd) {
					GameInp[nInput].Macro.pVal[i] = bii.pVal;
					GameInp[nInput].Macro.nInput[i] = j;

					GameInp[nInput].Macro.nVal[i] = (unsigned char)cstol(szEnd, &szValue, 0);

					nFound++;

					break;
				}
			}
		}

		if (nFound) {
			if (GameInp[nInput].Macro.pVal[nFound - 1]) {
				GameInp[nInput].nInput = GIT_MACRO_CUSTOM;
				GameInp[nInput].Macro.nMode = nMode;
				if (bCreateNew) {
					nMacroCount++;
				}
				return 0;
			}
		}
	}

	return 1;
}

int GameInputAutoIni(int nPlayer, char* lpszFile, bool bOverWrite)
{
	char szLine[1024];
	int nFileVersion = 0;
	unsigned int i;

	nAnalogSpeed = 0x0100;

	FILE* h = fopen(lpszFile, ("rt"));
	if (h == NULL) {
		return 1;
	}

	// Go through each line of the config file and process inputs
	while (_fgetts(szLine, sizeof(szLine), h)) {
		char* szValue;
		int nLen = cslen(szLine);

		// Get rid of the linefeed at the end
		if (szLine[nLen - 1] == 10) {
			szLine[nLen - 1] = 0;
			nLen--;
		}

		szValue = LabelCheck(szLine, ("version"));
		if (szValue) {
			nFileVersion = cstol(szValue, NULL, 0);
		}
		szValue = LabelCheck(szLine, ("analog"));
		if (szValue) {
			nAnalogSpeed = cstol(szValue, NULL, 0);
		}

		if (nConfigMinVersion <= nFileVersion && nFileVersion <= nBurnVer) {
			szValue = LabelCheck(szLine, ("input"));
			if (szValue) {
				char* szQuote = NULL;
				char* szEnd = NULL;
				if (QuoteRead(&szQuote, &szEnd, szValue)) {
					continue;
				}

				if ((szQuote[0] == ('p') || szQuote[0] == ('P')) && szQuote[1] >= ('1') && szQuote[1] <= ('0') + nMaxPlayers && szQuote[2] == (' ')) {
					if (szQuote[1] != ('1') + nPlayer) {
						continue;
					}
				} else {
					if (nPlayer != 0) {
						continue;
					}
				}

				// Find which input number this refers to
				i = InputNameToNum(szQuote);
				if (i == ~0U) {
					i = InputInfoToNum(szQuote);
					if (i == ~0U) {
						continue;
					}
				}

				if (GameInp[i].nInput == 0 || bOverWrite) {				// Undefined - assign mapping
					StringToInp(GameInp + i, szEnd);
				}
			}

			szValue = LabelCheck(szLine, ("macro"));
			if (szValue) {
				char* szQuote = NULL;
				char* szEnd = NULL;
				if (QuoteRead(&szQuote, &szEnd, szValue)) {
					continue;
				}

				i = MacroNameToNum(szQuote);
				if (i != ~0U) {
					i += nGameInpCount;
					if (GameInp[i].Macro.nMode == 0 || bOverWrite) {	// Undefined - assign mapping
						StringToMacro(GameInp + i, szEnd);
					}
				}
			}

			szValue = LabelCheck(szLine, ("custom"));
			if (szValue) {
				AddCustomMacro(szValue, bOverWrite);
			}
		}
	}

	fclose(h);

	return 0;
}

// Auto-configure any undefined inputs to defaults
int GameInpDefault()
{
	struct GameInp* pgi;
	struct BurnInputInfo bii;
	unsigned int i;

	for (int nPlayer = 0; nPlayer < nMaxPlayers; nPlayer++) {

		if ((nPlayerDefaultControls[nPlayer] & 0x0F) != 0x0F) {
			continue;
		}

		GameInputAutoIni(nPlayer, szPlayerDefaultIni[nPlayer], false);
	}

	// Fill all inputs still undefined
	for (i = 0, pgi = GameInp; i < nGameInpCount; i++, pgi++) {
		if (pgi->nInput) {											// Already defined - leave it alone
			continue;
		}

		// Get the extra info about the input
		bii.szInfo = NULL;
		BurnDrvGetInputInfo(&bii, i);
		if (bii.pVal == NULL) {
			continue;
		}
		if (bii.szInfo == NULL) {
			bii.szInfo = "";
		}

		// Dip switches - set to constant
		if (bii.nType & BIT_GROUP_CONSTANT) {
			pgi->nInput = GIT_CONSTANT;
			continue;
		}

		GameInpAutoOne(pgi, bii.szInfo);
	}

	// Fill in macros still undefined
	for (i = 0; i < nMacroCount; i++, pgi++) {
		if (pgi->nInput != GIT_MACRO_AUTO || pgi->Macro.nMode) {	// Already defined - leave it alone
			continue;
		}

		GameInpAutoOne(pgi, pgi->Macro.szName);
	}

	return 0;
}

// ---------------------------------------------------------------------------
// Write all the GameInps out to config file 'h'

int GameInpWrite(FILE* h)
{
	// Write input types
	for (unsigned int i = 0; i < nGameInpCount; i++) {
		char* szName = NULL;
		int nPad = 0;
		szName = InputNumToName(i);
		fprintf(h, ("input  \"%s\" "), szName);
		nPad = 16 - cslen(szName);
		for (int j = 0; j < nPad; j++) {
			fprintf(h, (" "));
		}
		fprintf(h, ("%s\n"), InpToString(GameInp + i));
	}

	fprintf(h, ("\n"));

	struct GameInp* pgi = GameInp + nGameInpCount;
	for (unsigned int i = 0; i < nMacroCount; i++, pgi++) {
		int nPad = 0;

		if (pgi->nInput & GIT_GROUP_MACRO) {
			switch (pgi->nInput) {
				case GIT_MACRO_AUTO:									// Auto-assigned macros
					fprintf(h, ("macro  \"%hs\" "), pgi->Macro.szName);
					break;
				case GIT_MACRO_CUSTOM:									// Custom macros
					fprintf(h, ("custom \"%hs\" "), pgi->Macro.szName);
					break;
				default:												// Unknown -- ignore
					continue;
			}

			nPad = 16 - strlen(pgi->Macro.szName);
			for (int j = 0; j < nPad; j++) {
				fprintf(h, (" "));
			}
			fprintf(h, ("%s\n"), InpMacroToString(pgi));
		}
	}

	return 0;
}

// ---------------------------------------------------------------------------

// Read a GameInp in
int GameInpRead(char* szVal, bool bOverWrite)
{
	int nRet;
	char* szQuote = NULL;
	char* szEnd = NULL;
	unsigned int i = 0;

	nRet = QuoteRead(&szQuote, &szEnd, szVal);
	if (nRet) {
		return 1;
	}

	// Find which input number this refers to
	i = InputNameToNum(szQuote);
	if (i == ~0U) {
		return 1;
	}

	if (bOverWrite || GameInp[i].nInput == 0) {
		// Parse the input description into the GameInp structure
		StringToInp(GameInp + i, szEnd);
	}

	return 0;
}

int GameInpMacroRead(char* szVal, bool bOverWrite)
{
	int nRet;
	char* szQuote = NULL;
	char* szEnd = NULL;
	unsigned int i = 0;

	nRet = QuoteRead(&szQuote, &szEnd, szVal);
	if (nRet) {
		return 1;
	}

	i = MacroNameToNum(szQuote);
	if (i != ~0U) {
		i += nGameInpCount;
		if (GameInp[i].Macro.nMode == 0 || bOverWrite) {
			StringToMacro(GameInp + i, szEnd);
		}
	}

	return 0;
}

int GameInpCustomRead(char* szVal, bool bOverWrite)
{
	return AddCustomMacro(szVal, bOverWrite);
}

*/
