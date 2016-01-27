#include <windows.h>
#include <mmsystem.h>
#include <commctrl.h>
#include <cmath>
#include <cstdio>
#include <string>
#include "resource.h" 

#pragma comment(lib, "winmm.lib")

HWND hwndScore;

double altitude = 3000;
double vertspeed = -35;
double acc;
int thrust = 40, deltathrust = 10;
int time = 0;
double fueluse;
double fuel = 100;
bool outoffuel = FALSE, alreadyrunning = FALSE, forwardplayed = FALSE, randomplayed = FALSE, secondsplayed = FALSE, autopilot = FALSE;
bool seventyfive = FALSE, fifty = FALSE, twentyfive = FALSE;
double score;

BOOL CALLBACK DlgProc(HWND hwnd, UINT Message, WPARAM wParam, LPARAM lParam)
{
	switch(Message)
	{

		case WM_INITDIALOG:
			char strVert[100];
			sprintf_s(strVert,"%0.1f", vertspeed);
			SetDlgItemText(hwnd, IDC_VERTSPEED, strVert);
			char strAlt[100];
			sprintf_s(strAlt,"%0.0f", altitude);
			SetDlgItemText(hwnd, IDC_ALTITUDE, strAlt);
			SetDlgItemInt(hwnd, IDC_ELAPSED, time, FALSE);
			SendDlgItemMessage(hwnd, IDC_FUEL, PBM_SETPOS, fuel, 0);
			SendDlgItemMessage(hwnd, IDC_THRUST, PBM_SETPOS, thrust, 0);
			SendDlgItemMessage(hwnd, IDC_CONTACTLIGHT, PBM_SETPOS, 0, 0);
			PlaySound(NULL, NULL, 0);
		break;

		case WM_CLOSE:
			EndDialog(hwnd, 0);
		break;

		case WM_COMMAND:
			switch(LOWORD(wParam))
			{
	
				case IDC_BEGIN:
					if (alreadyrunning == FALSE)
					{
					PlaySound((LPCSTR) IDW_GONOGO, NULL, SND_RESOURCE | SND_ASYNC );
					SetTimer(hwnd, IDT_MISSIONTIMER, 1000, NULL);
					SetTimer(hwnd, IDT_PHYSTIMER, 100, NULL);
					alreadyrunning = TRUE;
					}
					else MessageBox(hwnd, "Already running!", "Warning!", MB_OK);
				break;

			    case IDC_THRUSTUP:
					if (thrust < 100)
					{thrust = thrust + deltathrust;}
					else {thrust = 100;}
					SendDlgItemMessage(hwnd, IDC_THRUST, PBM_SETPOS, thrust, 0);
				break;

				case IDC_THRUSTDOWN:
					if (thrust > 0)
					{thrust = thrust - deltathrust;}
					else {thrust = 0;}
					SendDlgItemMessage(hwnd, IDC_THRUST, PBM_SETPOS, thrust, 0);
				break;

				case IDC_AUTOPILOT:
					if (autopilot == FALSE)
					{PlaySound((LPCSTR) IDW_APON, NULL, SND_RESOURCE | SND_ASYNC );
					autopilot = TRUE;}
					else if (autopilot == TRUE)
					{PlaySound((LPCSTR) IDW_APOFF, NULL, SND_RESOURCE | SND_ASYNC );
					autopilot = FALSE;}
				break;

				case IDC_RESET:
					KillTimer(hwnd, IDT_MISSIONTIMER);
					KillTimer(hwnd, IDT_PHYSTIMER);
					altitude = 3000;
					vertspeed = -35;
					thrust = 40;
					deltathrust = 10;
                    time = 0;
					fuel = 100;
					outoffuel = FALSE;
					alreadyrunning = FALSE;
					forwardplayed= FALSE;
					randomplayed = FALSE;
					secondsplayed = FALSE;
					autopilot = FALSE;
					seventyfive = FALSE;
					fifty = FALSE;
					twentyfive = FALSE;
					char strVert[100];
			        sprintf_s(strVert,"%0.1f", vertspeed);
			        SetDlgItemText(hwnd, IDC_VERTSPEED, strVert);
			        char strAlt[100];
			        sprintf_s(strAlt,"%0.0f", altitude);
			        SetDlgItemText(hwnd, IDC_ALTITUDE, strAlt);
			        SetDlgItemInt(hwnd, IDC_ELAPSED, time, FALSE);
			        SendDlgItemMessage(hwnd, IDC_FUEL, PBM_SETPOS, fuel, 0);
			        SendDlgItemMessage(hwnd, IDC_THRUST, PBM_SETPOS, thrust, 0);
					SendDlgItemMessage(hwnd, IDC_CONTACTLIGHT, PBM_SETPOS, 0, 0);
					PlaySound(NULL, NULL, 0);
			    break;  

				case IDC_ABOUT:
					MessageBox(hwnd, "By George Kristiansen\n\ngeorge7378@googlemail.com", "About", MB_OK);
			    break;
					
				case IDC_EXIT:
					KillTimer(hwnd, IDT_MISSIONTIMER);
					KillTimer(hwnd, IDT_PHYSTIMER);
					PlaySound(NULL, NULL, 0);
					EndDialog(hwnd, 0);
			    break; 

				case IDC_SCOREOK:
					EndDialog(hwndScore, 0);
				break;

			}
		break;

		case WM_TIMER:
			switch(LOWORD(wParam))
			{

			case IDT_PHYSTIMER:
				fueluse = thrust * 0.001;
				fuel = fuel - fueluse;
				acc = ((thrust * 487.2)+(15000*g))/15000;   //w = 24360
				altitude = altitude + ((vertspeed*0.1)+(0.5*acc*0.01));
				vertspeed = vertspeed + (acc * 0.1);

				if (altitude < 0.5 && vertspeed > -2)
				{
					autopilot = FALSE;
					altitude = 0;
					vertspeed = 0;
					thrust = 0;
					deltathrust = 0;
					KillTimer(hwnd, IDT_MISSIONTIMER);
					KillTimer(hwnd, IDT_PHYSTIMER);
					SendDlgItemMessage(hwnd, IDC_THRUST, PBM_SETPOS, 0, 0);
					SendDlgItemMessage(hwnd, IDC_CONTACTLIGHT, PBM_SETPOS, 100, 0);
					score = (fuel*1000)-(time*10);
					if (score < 0)
					{score = 0;}
                    char strScore[100];
			        sprintf_s(strScore,"%0.0f", score);
					hwndScore = CreateDialog(GetModuleHandle(NULL), MAKEINTRESOURCE(IDD_SCOREPAGE), hwnd, DlgProc);
			        SetDlgItemText(hwndScore, IDC_SCORE, strScore);
					PlaySound((LPCSTR) IDW_LANDED, NULL, SND_RESOURCE | SND_ASYNC );
				}

				else if (altitude < 0.5 && vertspeed <= -2)
				{
					autopilot = FALSE;
					altitude = 0;
					vertspeed = 0;
					thrust = 0;
					deltathrust = 0;
					KillTimer(hwnd, IDT_MISSIONTIMER);
					KillTimer(hwnd, IDT_PHYSTIMER);
					SendDlgItemMessage(hwnd, IDC_THRUST, PBM_SETPOS, 0, 0);
					PlaySound((LPCSTR) IDW_CRASH, NULL, SND_RESOURCE | SND_ASYNC );
					MessageBox(hwnd, "You landed too hard!", "Crashed!", MB_OK);
				}

				if (fuel <= 0 && outoffuel == FALSE)
				{
					fuel = 0;
					outoffuel = TRUE;
					autopilot = FALSE;
					deltathrust = 0;
					thrust = 0;
					SendDlgItemMessage(hwnd, IDC_THRUST, PBM_SETPOS, 0, 0);
					PlaySound((LPCSTR) IDW_APOFF, NULL, SND_RESOURCE | SND_ASYNC | SND_LOOP );
					MessageBox(hwnd, "Out of fuel! You're falling towards the Moon!", "Warning!", MB_OK);
				}

				if (altitude < 100 && secondsplayed == FALSE  && outoffuel == FALSE)
				{
					secondsplayed = TRUE;
					PlaySound((LPCSTR) IDW_60SECS, NULL, SND_RESOURCE | SND_ASYNC );
				}

				if (altitude < 50 && randomplayed == FALSE && outoffuel == FALSE)
				{
					randomplayed = TRUE;
					PlaySound((LPCSTR) IDW_RANDOM, NULL, SND_RESOURCE | SND_ASYNC );
				}

				if (altitude < 10 && forwardplayed == FALSE && outoffuel == FALSE)
				{
                    forwardplayed = TRUE;
                    PlaySound((LPCSTR) IDW_4FORWARD, NULL, SND_RESOURCE | SND_ASYNC );
				}

				if (fuel <= 75 && seventyfive == FALSE)
				{
					seventyfive = TRUE;
					PlaySound((LPCSTR) IDW_BEEP, NULL, SND_RESOURCE | SND_ASYNC );
				}

				if (fuel <= 50 && fifty == FALSE)
				{
					fifty = TRUE;
					PlaySound((LPCSTR) IDW_BEEP, NULL, SND_RESOURCE | SND_ASYNC );
				}

				if (fuel <= 25 && twentyfive == FALSE)
				{
					twentyfive = TRUE;
					PlaySound((LPCSTR) IDW_BEEP, NULL, SND_RESOURCE | SND_ASYNC );
				}

				if (autopilot == TRUE && outoffuel == FALSE)
				{
					if (altitude > 2000 && vertspeed < -40)
					{thrust = 90;}
					else if (altitude > 2000 && vertspeed >= -40 && vertspeed <= -39.8)
					{thrust = 50;}
				    else if (altitude > 2000 && vertspeed > -39.8)
					{thrust = 10;}

					if (altitude > 700 && altitude <= 2000 && vertspeed < -30)
					{thrust = 80;}
					else if (altitude > 700 && altitude <= 2000 && vertspeed >= -30 && vertspeed <= -29.8)
					{thrust = 50;}
					else if (altitude > 700 && altitude <= 2000 && vertspeed > -29.8)
					{thrust = 10;}

					if (altitude > 250 && altitude <= 700 && vertspeed < -20)
					{thrust = 80;}
					else if (altitude > 250 && altitude <= 700 && vertspeed >= -20 && vertspeed <= -19.8)
					{thrust = 50;}
					else if (altitude > 250 && altitude <= 700 && vertspeed > -19.8)
					{thrust = 10;}

					if (altitude > 100 && altitude <= 250 && vertspeed < -10)
					{thrust = 100;}
					else if (altitude > 100 && altitude <= 250 && vertspeed >= -10 && vertspeed <= -9.8)
					{thrust = 50;}
					else if (altitude > 100 && altitude <= 250 && vertspeed > -9.8)
					{thrust = 10;}

					if (altitude > 30 && altitude <= 100 && vertspeed < -5)
					{thrust = 100;}
					else if (altitude > 30 && altitude <= 100 && vertspeed >= -5 && vertspeed <= -4.8)
					{thrust = 50;}
					else if (altitude > 30 && altitude <= 100 && vertspeed > -4.8)
					{thrust = 10;}

					if (altitude > 5 && altitude <= 30 && vertspeed < -2)
					{thrust = 100;}
					else if (altitude > 5 && altitude <= 30 && vertspeed >= -2 && vertspeed <= -1.8)
					{thrust = 50;}
					else if (altitude > 5 && altitude <= 30 && vertspeed > -1.8)
					{thrust = 10;}

					if (altitude <= 5 && vertspeed < -0.5)
					{thrust = 80;}
					else if (altitude <= 5 && vertspeed >= -0.5 && vertspeed <= -0.3)
					{thrust = 50;}
					else if (altitude <= 5 && vertspeed > -0.3)
					{thrust = 10;}

					SendDlgItemMessage(hwnd, IDC_THRUST, PBM_SETPOS, thrust, 0);
				}

				char strVert[100];
			    sprintf_s(strVert,"%0.1f", vertspeed);
			    SetDlgItemText(hwnd, IDC_VERTSPEED, strVert);
				char strAlt[100];
			    sprintf_s(strAlt,"%0.0f", altitude);
			    SetDlgItemText(hwnd, IDC_ALTITUDE, strAlt);
			    SetDlgItemInt(hwnd, IDC_ELAPSED, time, FALSE);
			    SendDlgItemMessage(hwnd, IDC_FUEL, PBM_SETPOS, fuel, 0);
			break;
				

			case IDT_MISSIONTIMER:
				time = time + 1;
				SetDlgItemInt(hwnd, IDC_ELAPSED, time, FALSE);
			break;

			}
        break;

		case WM_DESTROY:
			KillTimer(hwnd, IDT_MISSIONTIMER);
			KillTimer(hwnd, IDT_PHYSTIMER);
			PlaySound(NULL, NULL, 0);
		break;

		default:
			return FALSE;
	}
	return TRUE;
}

int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance,
	LPSTR lpCmdLine, int nCmdShow)
{
	INITCOMMONCONTROLSEX initex;
	initex.dwSize = sizeof(INITCOMMONCONTROLSEX);
	initex.dwICC  = ICC_PROGRESS_CLASS;
	InitCommonControlsEx(&initex);
	return DialogBox(hInstance, MAKEINTRESOURCE(IDD_MAIN), NULL, DlgProc);
}