#include "burner.h"

static bool SkipComma(char** s)
{
	while (**s && **s != (',')) {
		(*s)++;
	}

	if (**s == (',')) {
		(*s)++;
	}

	if (**s) {
		return true;
	}

	return false;
}

static void CheatError(char* pszFilename, int nLineNumber, CheatInfo* pCheat, char* pszInfo, char* pszLine)
{
	//FBAPopupAddText(PUFEXT_NORANSLATE, ("Cheat file %s is malformed.\nPlease remove or repair the file.\n\n"), pszFilename);
	if (pCheat) {
		//FBAPopupAddText(PUFEXT_NORANSLATE, ("Parse error at line %i, in cheat \"%s\".\n"), nLineNumber, pCheat->szCheatName);
	} else {
		//FBAPopupAddText(PUFEXT_NORANSLATE, ("Parse error at line %i.\n"), nLineNumber);
	}

	if (pszInfo) {
		//FBAPopupAddText(PUFEXT_NORANSLATE, ("Problem:\t%s.\n"), pszInfo);
	}
	if (pszLine) {
		//FBAPopupAddText(PUFEXT_NORANSLATE, ("Text:\t%s\n"), pszLine);
	}

	//FBAPopupDisplay(PUFYPE_ERROR);
}

static int ConfigParseFile(char* pszFilename)
{
#define INSIDE_NOTHING (0xFFFF & (1 << (sizeof(char) * 8) - 1))

	char szLine[1024];
	char* s;
	char* t;
	int nLen;

	int nLine = 0;
	char nInside = INSIDE_NOTHING;

	CheatInfo* pCurrentCheat = NULL;

	FILE* h = fopen(pszFilename, ("rt"));
	if (h == NULL) {
		return 1;
	}

	while (1) {
		if (fgets(szLine, sizeof(szLine), h) == NULL) {  //_fgetts(szLine, sizeof(szLine), h) == NULL) {
			break;
		}

		nLine++;

		nLen = strlen(szLine);
		// Get rid of the linefeed at the end
		while (szLine[nLen - 1] == 0x0A || szLine[nLen - 1] == 0x0D) {
			szLine[nLen - 1] = 0;
			nLen--;
		}

		s = szLine;													// Start parsing

		if (s[0] == ('/') && s[1] == ('/')) {					// Comment
			continue;
		}

		if ((t = LabelCheck(s, ("include"))) != 0) {				// Include a file
			s = t;

			char szFilename[MAX_PATH] = ("");

			// Read name of the cheat file
			char* szQuote = NULL;
			QuoteRead(&szQuote, NULL, s);

			sprintf(szFilename, ("cheats\\%s.dat"), szQuote);

			if (ConfigParseFile(szFilename)) {
				sprintf(szFilename, ("cheats\\%s.ini"), szQuote);
				if (ConfigParseFile(szFilename)) {
					CheatError(pszFilename, nLine, NULL, ("included file doesn't exist"), szLine);
				}
			}

			continue;
		}

		if ((t = LabelCheck(s, ("cheat"))) != 0) {				// Add new cheat
			s = t;

			// Read cheat name
			char* szQuote = NULL;
			char* szEnd = NULL;

			QuoteRead(&szQuote, &szEnd, s);

			s = szEnd;

			if ((t = LabelCheck(s, ("advanced"))) != 0) {			// Advanced cheat
				s = t;
			}

			//SKIP_WS(s);

			if (nInside == ('{')) {
				CheatError(pszFilename, nLine, pCurrentCheat, ("missing closing bracket"), NULL);
				break;
			}
#if 0
			if (*s != ('\0') && *s != ('{')) {
				CheatError(pszFilename, nLine, NULL, ("malformed cheat declaration"), szLine);
				break;
			}
#endif
			nInside = *s;

			// Link new node into the list
			CheatInfo* pPreviousCheat = pCurrentCheat;
			pCurrentCheat = (CheatInfo*)malloc(sizeof(CheatInfo));
			if (pCheatInfo == NULL) {
				pCheatInfo = pCurrentCheat;
			}

			memset(pCurrentCheat, 0, sizeof(CheatInfo));
			pCurrentCheat->pPrevious = pPreviousCheat;
			if (pPreviousCheat) {
				pPreviousCheat->pNext = pCurrentCheat;
			}

			// Fill in defaults
			pCurrentCheat->nType = 0;								// Default to cheat type 0 (apply each frame)
			pCurrentCheat->nStatus = -1;							// Disable cheat

			memcpy(pCurrentCheat->szCheatName, szQuote, QUOTE_MAX);

			continue;
		}

		if ((t = LabelCheck(s, ("type"))) != 0) {					// Cheat type
#if defined (UNICODE)
			if (nInside == INSIDE_NOTHING || pCurrentCheat == NULL) {
				CheatError(pszFilename, nLine, pCurrentCheat, ("rogue cheat type"), szLine);
				break;
			}
#endif
			s = t;

			// Set type
			pCurrentCheat->nType = (int)*s;//cstol(s, NULL, 0);

			continue;
		}

		if ((t = LabelCheck(s, ("default"))) != 0) {				// Default option
#if defined (UNICODE)
			if (nInside == INSIDE_NOTHING || pCurrentCheat == NULL) {
				CheatError(pszFilename, nLine, pCurrentCheat, ("rogue default"), szLine);
				break;
			}
#endif
			s = t;

			// Set default option
			pCurrentCheat->nDefault = (int)&s;//cstol(s, NULL, 0);

			continue;
		}

		int n = (int)*s; // ?? //cstol(s, &t, 0);
		if (t != s) {				   								// New option

#if defined (UNICODE)
			if (nInside == INSIDE_NOTHING || pCurrentCheat == NULL) {
				CheatError(pszFilename, nLine, pCurrentCheat, ("rogue option"), szLine);
				break;
			}
#endif

			// Link a new Option structure to the cheat
			if (n < CHEAT_MAX_OPTIONS) {
				s = t;

				// Read option name
				char* szQuote = NULL;
				char* szEnd = NULL;
				if (QuoteRead(&szQuote, &szEnd, s)) {
					CheatError(pszFilename, nLine, pCurrentCheat, ("option name omitted"), szLine);
					break;
				}
				s = szEnd;

				if (pCurrentCheat->pOption[n] == NULL) {
					pCurrentCheat->pOption[n] = (CheatOption*)malloc(sizeof(CheatOption));
				}
				memset(pCurrentCheat->pOption[n], 0, sizeof(CheatOption));

				memcpy(pCurrentCheat->pOption[n]->szOptionName, szQuote, QUOTE_MAX * sizeof(char));

				int nCurrentAddress = 0;
				bool bOK = true;
				while (nCurrentAddress < CHEAT_MAX_ADDRESS) {
					int nCPU = 0, nAddress = 0, nValue = 0;

					if (SkipComma(&s)) {
						//nCPU = cstol(s, &t, 0);		// CPU number
						nCPU = (int)*s;						
						if (t == s) {
							CheatError(pszFilename, nLine, pCurrentCheat, ("CPU number omitted"), szLine);
							bOK = false;
							break;
						}
						s = t;

						SkipComma(&s);
						//nAddress = cstol(s, &t, 0);	// Address
						nAddress = (int)*s;						
						if (t == s) {
							bOK = false;
							CheatError(pszFilename, nLine, pCurrentCheat, ("address omitted"), szLine);
							break;
						}
						s = t;

						SkipComma(&s);
						//nValue = cstol(s, &t, 0);		// Value
						nValue = (int)*s;						
						if (t == s) {
							bOK = false;
							CheatError(pszFilename, nLine, pCurrentCheat, ("value omitted"), szLine);
							break;
						}
					} else {
						if (nCurrentAddress) {			// Only the first option is allowed no address
							break;
						}
						if (n) {
							bOK = false;
							CheatError(pszFilename, nLine, pCurrentCheat, ("CPU / address / value omitted"), szLine);
							break;
						}
					}

					pCurrentCheat->pOption[n]->AddressInfo[nCurrentAddress].nCPU = nCPU;
					pCurrentCheat->pOption[n]->AddressInfo[nCurrentAddress].nAddress = nAddress;
					pCurrentCheat->pOption[n]->AddressInfo[nCurrentAddress].nValue = nValue;
					nCurrentAddress++;
				}

				if (!bOK) {
					break;
				}

			}

			continue;
		}

		//SKIP_WS(s);
		if (*s == ('}')) {
			if (nInside != ('{')) {
				CheatError(pszFilename, nLine, pCurrentCheat, ("missing opening bracket"), NULL);
				break;
			}

			nInside = INSIDE_NOTHING;
		}

		// Line isn't (part of) a valid cheat
#if 0
		if (*s) {
			CheatError(pszFilename, nLine, NULL, ("rogue line"), szLine);
			break;
		}
#endif

	}

	if (h) {
		fclose(h);
	}

	return 0;
}

int ConfigCheatLoad()
{
	char szFilename[MAX_PATH] = ("");

	//_stprintf(szFilename, ("cheats\\%s.dat"), BurnDrvGetText(DRV_NAME));
	sprintf(szFilename, "cheats\\%s.dat", BurnDrvGetText(DRV_NAME));
	if (ConfigParseFile(szFilename)) {
		//_stprintf(szFilename, ("cheats\\%s.ini"), BurnDrvGetText(DRV_NAME));
		sprintf(szFilename, "cheats\\%s.ini", BurnDrvGetText(DRV_NAME));
		if (ConfigParseFile(szFilename)) {
			return 1;
		}
	}

	if (pCheatInfo) {
		int nCurrentCheat = 0;
		while (CheatEnable(nCurrentCheat, -1) == 0) {
			nCurrentCheat++;
		}

		CheatUpdate();
	}

	return 0;
}


