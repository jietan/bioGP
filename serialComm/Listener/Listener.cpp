//	Listener.cpp - Sample application for CSerial
//
//	Copyright (C) 1999-2003 Ramon de Klein (Ramon.de.Klein@ict.nl)
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.

#define STRICT
#include <tchar.h>
#include <windows.h>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <vector>
#include "timer.h"
#include "Serial.h"


#define NUM_MOTORS 16
#define NUM_BYTES_PER_MOTOR 5

enum { EOF_Char = 27 };


int MakeWord(int hex0, int hex1, int hex2, int hex3)
{
	unsigned int word;

	word = ((hex0 << 12) + (hex1 << 8) + (hex2 << 4) + hex3);
	return (int)word;
}

int ShowError (LONG lError, LPCTSTR lptszMessage)
{
	// Generate a message text
	TCHAR tszMessage[256];
	wsprintf(tszMessage,_T("%s\n(error code %d)"), lptszMessage, lError);

	// Display message-box and return with an error-code
	::MessageBox(0,tszMessage,_T("Listener"), MB_ICONSTOP|MB_OK);
	return 1;
}

int Decipher(char c)
{
	int ret = c;
	if (ret > '9')
	{
		ret -= 7;
	}
	ret -= '0';
	return ret;
}

bool ProcessFrame(const string& frame, int* motorAngle)
{
	std::vector<int> tmpPos;
	tmpPos.resize(NUM_MOTORS);
	for (int i = 0; i < NUM_MOTORS; ++i)
	{
		int mPos = MakeWord(Decipher(frame[NUM_BYTES_PER_MOTOR * i]), Decipher(frame[NUM_BYTES_PER_MOTOR * i + 1]), Decipher(frame[NUM_BYTES_PER_MOTOR * i + 2]), Decipher(frame[NUM_BYTES_PER_MOTOR * i + 3]));
		tmpPos[i] = mPos;
		if (mPos < 0 || mPos >= 1024)
		{
			std::cout << "Warning! Joint angle abnormal." << std::endl;
			return false;
		}
		
	}
	for (int i = 0; i < NUM_MOTORS; ++i)
	{
		motorAngle[i] = tmpPos[i];
	}
	return true;
}

bool ProcessBuffer(string& buff, int* motorAngle)
{
	bool ret = false;
	size_t endPos = buff.find_last_of("\n", buff.size());
	if (endPos >= buff.size())
	{
		//std::cout << "Warning! End symbol not found!" << std::endl;
		return false;
	}
	else if (endPos == NUM_BYTES_PER_MOTOR * NUM_MOTORS + 1 - 1)
	{
		ret = ProcessFrame(buff, motorAngle);
	}
	else if (endPos < NUM_BYTES_PER_MOTOR * NUM_MOTORS + 1 - 1)
	{
	}
	else
	{
		size_t newEndPos = buff.find_last_of("\n", endPos - 1);
		if (endPos - newEndPos == NUM_BYTES_PER_MOTOR * NUM_MOTORS + 1)
		{
			string frame = buff.substr(newEndPos + 1, endPos);
			ret = ProcessFrame(frame, motorAngle);
		}
	}
	buff = buff.substr(endPos + 1, buff.size());

	return ret;
}

int __cdecl _tmain (int /*argc*/, char** /*argv*/)
{
    CSerial serial;
	LONG    lLastError = ERROR_SUCCESS;

    // Attempt to open the serial port (COM1)
    lLastError = serial.Open(_T("COM3"),0,0,false);
	if (lLastError != ERROR_SUCCESS)
		return ::ShowError(serial.GetLastError(), _T("Unable to open COM-port"));

    // Setup the serial port (9600,8N1, which is the default setting)
	lLastError = serial.Setup(CSerial::EBaud57600, CSerial::EData8, CSerial::EParNone, CSerial::EStop1);
	if (lLastError != ERROR_SUCCESS)
		return ::ShowError(serial.GetLastError(), _T("Unable to set COM-port setting"));

    // Register only for the receive event
    lLastError = serial.SetMask(CSerial::EEventBreak |
								CSerial::EEventCTS   |
								CSerial::EEventDSR   |
								CSerial::EEventError |
								CSerial::EEventRing  |
								CSerial::EEventRLSD  |
								CSerial::EEventRecv);
	if (lLastError != ERROR_SUCCESS)
		return ::ShowError(serial.GetLastError(), _T("Unable to set COM-port event mask"));

	// Use 'non-blocking' reads, because we don't know how many bytes
	// will be received. This is normally the most convenient mode
	// (and also the default mode for reading data).
    lLastError = serial.SetupReadTimeouts(CSerial::EReadTimeoutNonblocking);
	if (lLastError != ERROR_SUCCESS)
		return ::ShowError(serial.GetLastError(), _T("Unable to set COM-port read timeout."));

    // Keep reading data, until an EOF (CTRL-Z) has been received
	bool fContinue = true;
	
	int motorAngle[NUM_MOTORS];
	memset(motorAngle, 0, NUM_MOTORS * sizeof(int));

	lLastError = serial.Write("w");
	if (lLastError != ERROR_SUCCESS)
		return ::ShowError(serial.GetLastError(), _T("Unable to send data"));
	DecoTimer timer;

	timer.startTimer();
	std::string tmpBuffer = "";
	int updateCount = 0;
	int totalCount = 0;
	do
	{
		// Wait for an event
		lLastError = serial.WaitEvent();

		if (lLastError != ERROR_SUCCESS)
			return ::ShowError(serial.GetLastError(), _T("Unable to wait for a COM-port event."));

		// Save event
		const CSerial::EEvent eEvent = serial.GetEventType();

		// Handle break event
		if (eEvent & CSerial::EEventBreak)
		{
			printf("\n### BREAK received ###\n");
		}

		// Handle CTS event
		if (eEvent & CSerial::EEventCTS)
		{
			printf("\n### Clear to send %s ###\n", serial.GetCTS()?"on":"off");
		}

		// Handle DSR event
		if (eEvent & CSerial::EEventDSR)
		{
			printf("\n### Data set ready %s ###\n", serial.GetDSR()?"on":"off");
		}

		// Handle error event
		if (eEvent & CSerial::EEventError)
		{
			printf("\n### ERROR: ");
			switch (serial.GetError())
			{
			case CSerial::EErrorBreak:		printf("Break condition");			break;
			case CSerial::EErrorFrame:		printf("Framing error");			break;
			case CSerial::EErrorIOE:		printf("IO device error");			break;
			case CSerial::EErrorMode:		printf("Unsupported mode");			break;
			case CSerial::EErrorOverrun:	printf("Buffer overrun");			break;
			case CSerial::EErrorRxOver:		printf("Input buffer overflow");	break;
			case CSerial::EErrorParity:		printf("Input parity error");		break;
			case CSerial::EErrorTxFull:		printf("Output buffer full");		break;
			default:						printf("Unknown");					break;
			}
			printf(" ###\n");
		}

		// Handle ring event
		if (eEvent & CSerial::EEventRing)
		{
			printf("\n### RING ###\n");
		}

		// Handle RLSD/CD event
		if (eEvent & CSerial::EEventRLSD)
		{
			printf("\n### RLSD/CD %s ###\n", serial.GetRLSD()?"on":"off");
		}

		// Handle data receive event
		if (eEvent & CSerial::EEventRecv)
		{
			// Read data, until there is nothing left
			DWORD dwBytesRead = 0;
			char szBuffer[101];
			do
			{
				// Read data from the COM-port
				lLastError = serial.Read(szBuffer,sizeof(szBuffer)-1,&dwBytesRead);

				if (lLastError != ERROR_SUCCESS)
					return ::ShowError(serial.GetLastError(), _T("Unable to read from COM-port."));

				if (dwBytesRead > 0)
				{
					tmpBuffer.insert(tmpBuffer.size(), szBuffer, dwBytesRead);
					
					if (tmpBuffer.size() >= NUM_BYTES_PER_MOTOR * NUM_MOTORS + 1)
					{

						bool bUpdated = ProcessBuffer(tmpBuffer, motorAngle);

						totalCount++;
						if (bUpdated)
						{
							for (int i = 0; i < NUM_MOTORS; ++i)
							{
								std::cout << motorAngle[i] << " ";
							}
							std::cout << std::endl;
							//double duration = timer.getCurrentTime();
							//std::cout << "Takes " << duration << " seconds." << std::endl;
							//timer.resetTimer();
							//updateCount++;
							//std::cout << "Hit rate: " << static_cast<float>(updateCount) / totalCount;
						}

					}


				}

			}
		    while (dwBytesRead == sizeof(szBuffer)-1);
		}
	}
	while (fContinue);

    // Close the port again
    serial.Close();
    return 0;
}
