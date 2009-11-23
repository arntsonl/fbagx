#include "burner.h"

char* gameAv = NULL;
bool avOk = false;

static void CreateRomDatName(TCHAR* szRomDat)
{
	_stprintf(szRomDat, _T("sd:\\fbagx\\config\\roms.dat"));

	return;
}

int RomsDirCreate(/*HWND hParentWND*/)
{
	//hParent = hParentWND;
	
	//FBADialogBox(hAppInst, MAKEINTRESOURCE(IDD_ROMSDIR), hParent, DefInpProc);
	return 1;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
//Check Romsets Dialog/////////////////////////////////////////////////////////////////////////////

static int WriteGameAvb()
{
	TCHAR szRomDat[MAX_PATH];
	FILE* h;

	CreateRomDatName(szRomDat);

	if ((h = fopen(szRomDat, _T("wt"))) == NULL) {
		return 1;
	}

	_ftprintf(h, _T(APP_TITLE) _T(" v0.Wii ROMs"));	// identifier
	_ftprintf(h, _T(" 0x%04X "), nBurnDrvCount);					// no of games

	for (unsigned int i = 0; i < nBurnDrvCount; i++) {
		if (gameAv[i] & 2) {
			_fputtc(_T('*'), h);
		} else {
			if (gameAv[i] & 1) {
				_fputtc(_T('+'), h);
			} else {
				_fputtc(_T('-'), h);
			}
		}
	}

	_ftprintf(h, _T(" END"));									// end marker

	fclose(h);

	return 0;
}

static int DoCheck(TCHAR* buffPos)
{
	TCHAR label[256];

	// Check identifier
	memset(label, 0, sizeof(label));
	_stprintf(label, _T(APP_TITLE) _T(" v0.Wii ROMs"));
	if ((buffPos = LabelCheck(buffPos, label)) == NULL) {
		return 1;
	}

	// Check no of supported games
	memset(label, 0, sizeof(label));
	memcpy(label, buffPos, 16);
	buffPos += 8;
	unsigned int n = _tcstol(label, NULL, 0);
	if (n != nBurnDrvCount) {
		return 1;
	}

	for (unsigned int i = 0; i < nBurnDrvCount; i++) {
		if (*buffPos == _T('*')) {
			gameAv[i] = 3;
		} else {
			if (*buffPos == _T('+')) {
				gameAv[i] = 1;
			} else {
				if (*buffPos == _T('-')) {
					gameAv[i] = 0;
				} else {
					return 1;
				}
			}
		}

		buffPos++;
	}

	memset(label, 0, sizeof(label));
	_stprintf(label, _T(" END"));
	if (LabelCheck(buffPos, label) == NULL) {
		avOk = true;
		return 0;
	} else {
		return 1;
	}
}

int CheckGameAvb()
{
	TCHAR szRomDat[MAX_PATH];
	FILE* h;
	int bOK;
	int nBufferSize = nBurnDrvCount + 256;
	TCHAR* buffer = (TCHAR*)malloc(nBufferSize * sizeof(TCHAR));
	if (buffer == NULL) {
		return 1;
	}

	memset(buffer, 0, nBufferSize * sizeof(TCHAR));
	CreateRomDatName(szRomDat);

	if ((h = fopen(szRomDat, _T("r"))) == NULL) {
		return 1;
	}

	_fgetts(buffer, nBufferSize, h);
	fclose(h);

	bOK = DoCheck(buffer);

	free(buffer);
	return bOK;
}

static int QuitRomsScan()
{
/*
	DWORD dwExitCode;

	GetExitCodeThread(hScanThread, &dwExitCode);

	if (dwExitCode == STILL_ACTIVE) {

		// Signal the scan thread to abort
		SetEvent(hEvent);

		// Wait for the thread to finish
		if (WaitForSingleObject(hScanThread, 10000) != WAIT_OBJECT_0) {
			// If the thread doesn't finish within 10 seconds, forcibly kill it
			TerminateThread(hScanThread, 1);
		}

		CloseHandle(hScanThread);
	}

	CloseHandle(hEvent);

	hEvent = NULL;

	hScanThread = NULL;
	dwScanThreadId = 0;
*/
	BzipClose();

//	nBurnDrvSelect = nOldSelect;
//	nOldSelect = 0;
//	bRescanRoms = false;

	if (avOk) {
		WriteGameAvb();
	}

	return 1;
}

int AnalyzeRoms(void)					// LPVOID lParam
{
	for (unsigned int z = 0; z < nBurnDrvCount; z++) {
		nBurnDrvSelect = z;
/*
		// See if we need to abort
		if (WaitForSingleObject(hEvent, 0) == WAIT_OBJECT_0) {
			ExitThread(0);
		}

		SendDlgItemMessage(hRomsDlg, IDC_WAIT_PROG, PBM_STEPIT, 0, 0);
*/
		switch (BzipOpen(true))	{
			case 0:
				gameAv[z] = 3;
				break;
			case 2:
				gameAv[z] = 1;
				break;
			case 1:
				gameAv[z] = 0;
		}
		BzipClose();
   }

	avOk = true;

//	PostMessage(hRomsDlg, WM_CLOSE, 0, 0);

	return 0;
}
/*
static BOOL CALLBACK WaitProc(HWND hDlg, UINT Msg, WPARAM wParam, LPARAM)		// LPARAM lParam
{
	switch (Msg) {
		case WM_INITDIALOG:
			hRomsDlg = hDlg;
			nOldSelect = nBurnDrvSelect;
			memset(gameAv, 0, sizeof(gameAv));
			SendDlgItemMessage(hDlg, IDC_WAIT_PROG, PBM_SETRANGE, 0, MAKELPARAM(0, nBurnDrvCount));
			SendDlgItemMessage(hDlg, IDC_WAIT_PROG, PBM_SETSTEP, (WPARAM)1, 0);

			ShowWindow(GetDlgItem(hDlg, IDC_WAIT_LABEL_A), TRUE);
			SendMessage(GetDlgItem(hDlg, IDC_WAIT_LABEL_A), WM_SETTEXT, (WPARAM)0, (LPARAM)_T("Scanning ROMs..."));
			ShowWindow(GetDlgItem(hDlg, IDCANCEL), TRUE);

			avOk = false;
			hScanThread = CreateThread(NULL, 0, AnalyzingRoms, NULL, THREAD_TERMINATE, &dwScanThreadId);

			hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
			
			if (hParent == NULL) {
				RECT rect;
				int x, y;

				SystemParametersInfo(SPI_GETWORKAREA, 0, &rect, 0);

				x = 315 + GetSystemMetrics(SM_CXDLGFRAME) * 2 + 6;
				y = 74 + GetSystemMetrics(SM_CYDLGFRAME) * 2 + 6;

				SetForegroundWindow(hDlg);
				SetWindowPos(hDlg, HWND_TOPMOST, (rect.right - rect.left) / 2 - x / 2, (rect.bottom - rect.top) / 2 - y / 2, x, y, 0);
				RedrawWindow(hDlg, NULL, NULL, 0);
				ShowWindow(hDlg, SW_SHOWNORMAL);
			} else {
				WndInMid(hDlg, hParent);
				SetFocus(hDlg);		// Enable Esc=close
			}

			break;

		case WM_COMMAND:
			if (LOWORD(wParam) == IDCANCEL) {
				PostMessage(hDlg, WM_CLOSE, 0, 0);
			}
			break;

		case WM_CLOSE:
			QuitRomsScan();
			EndDialog(hDlg, 0);
			hRomsDlg = NULL;
			hParent = NULL;

	}

	return 0;
}
*/
int CreateROMInfo()
{
	if (gameAv == NULL) {
		gameAv = (char*)malloc(nBurnDrvCount);
		memset(gameAv, 0, nBurnDrvCount);
	}

	if (gameAv) {
		if (CheckGameAvb()/* || bRescanRoms*/) {
			//FBADialogBox(hAppInst, MAKEINTRESOURCE(IDD_WAIT), hParent, WaitProc);
		}
	}

	return 1;
}

void FreeROMInfo()
{
	free(gameAv);
	gameAv = NULL;
}
