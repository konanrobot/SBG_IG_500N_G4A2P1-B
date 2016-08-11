/**
 *	\file		ig500Continuous.c
 *  \author		SBG Systems (Raphael SIRYANI)
 *	\date		16/12/08.
 *
 *	This minimal project illustrates how simple
 *  it is to use the continuous mode for IG devices.
 */

#include "stdio.h"
#include "time.h"
#include <stdlib.h>
#include "sbgCom.h"
#include "protocol/commandsOutput.h"

/*!
 *	Functon called each time we have an error on a continuous frame.
 *	\param[in]	pHandler			Our sbgCom protocol handler.
 *	\param[in]	errorCode			Error code that have occured during a continuous operation.
 *	\param[in]	pUsrArg				User argument pointer as defined in sbgSetContinuousErrorCallback function.
 */
void continuousErrorCallback(SbgProtocolHandleInt *pHandler, SbgErrorCode errorCode, void *pUsrArg)
{
	char errorMsg[256];

	//
	// Convert our error code to a human readable error message
	//
	sbgComErrorToString(errorCode, errorMsg);

	//
	// Display an error message
	//
	fprintf(stderr,"continuousErrorCallback: We have received the following error %s\n", errorMsg);
	fflush(stderr);
}

/*!
 *	Functon called each time we have received a new data on a continuous frame.
 *	\param[in]	pHandler			Our sbgCom protocol handler.
 *	\param[in]	pOutput				Pointer on our received data struct. (You don't have the ownership so don't delete it!)
 *	\param[in]	pUsrArg				User argument pointer as defined in sbgSetContinuousModeCallback function.
 */
void continuousCallback(SbgProtocolHandleInt *handler, SbgOutput *pOutput, void *pUsrArg)
{
	if (pOutput)
	{
		/*
		printf("%3.2f\t%3.2f\t%3.2f\n",	SBG_RAD_TO_DEG(pOutput->stateEuler[0]),
										SBG_RAD_TO_DEG(pOutput->stateEuler[1]),
										SBG_RAD_TO_DEG(pOutput->stateEuler[2]));
		*/


        printf("%3.2f\t%3.2f\t%3.2f\n",	pOutput->accelerometers[0],
										pOutput->accelerometers[1],
										pOutput->accelerometers[2]);

		printf("%3.2f\t%3.2f\t%3.2f\n",	pOutput->gyroscopes[0],
										pOutput->gyroscopes[1],
										pOutput->gyroscopes[2]);
	}
}

//----------------------------------------------------------------------//
//  Main program                                                        //
//----------------------------------------------------------------------//
int main(int argc, char** argv)
{
	SbgProtocolHandle protocolHandle;

	//
	// Init our communications with the device (Please change here the com port and baud rate
	// On MacOS X, use for example "/dev/cu.SLAB_USBtoUART"
	// On Windows, use for example "COM3"
	//
	if (sbgComInit("/dev/ttyUSB0", 115200, &protocolHandle) == SBG_NO_ERROR)
	{
		//
		// Wait until the device has been initialised
		//
		sbgSleep(50);

		//
		// Display the title
		//
		printf("Euler Angles:\n");

		//
		// Define the default output mask for continuous mode
		// This setting is not saved in flash non volatile memory
		//

		if (sbgSetDefaultOutputMask(protocolHandle, FALSE,	SBG_OUTPUT_ACCELEROMETERS | SBG_OUTPUT_GYROSCOPES) != SBG_NO_ERROR)
		{
			fprintf(stderr,"Unable to set default output mask\n");
			fflush(stderr);
		}

		// This command actually enables continuous mode
		// Setting is not saved in flash memory
		//
		if (sbgSetContinuousMode(protocolHandle, FALSE, TRUE, 0) != SBG_NO_ERROR)
		{
			fprintf(stderr,"Unable to enable continuous mode\n");
			fflush(stderr);
		}

		//
		// Define our continuous handlers
		//
		sbgSetContinuousErrorCallback(protocolHandle, continuousErrorCallback, NULL);
		sbgSetContinuousModeCallback(protocolHandle, continuousCallback, NULL);

		//
		// Main loop
		//
		while (1)
		{
			//
			// Handle our continuous mode system
			//
			sbgProtocolContinuousModeHandle(protocolHandle);

			//
			// Small pause to unload CPU
			//
			sbgSleep(10);
		}

		//
		// Close our protocol system
		//
		sbgProtocolClose(protocolHandle);

		return 0;
	}
	else
	{
		fprintf(stderr, "Unable to open IG-500 device\n");
		fflush(stderr);
		return -1;
	}
}
