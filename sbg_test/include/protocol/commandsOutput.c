#include "commandsOutput.h"
#include "commands.h"
#include "protocolOutputMode.h"

/*!
 *	Define continous mode options, enabled/disabled and output frames speed.
 *	\param[in]	handle							A valid sbgCom library handle.
 *	\param[in]	permanent						If true, stores this settings in flash memory.
 *	\param[in]	contMode						Defines which continuous mode is set. Possible values are: <br>
 *												- SBG_CONT_TRIGGER_MODE_DISABLE <br>
 *												- SBG_CONTINUOUS_MODE_ENABLE <br>
 *												- SBG_TRIGGERED_MODE_ENABLE <br>
 *	\param[in]	divider							Main loop frequency divider applied on output.<br>
 *												For example if the device filter frequency has been set to 100Hz and we are using a divider of 4,<br>
 *												we will have data outputted at 100/4 = 25Hz.
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgSetContinuousMode(SbgProtocolHandle handle, bool permanent, SbgContOutputTypes contMode, uint8 divider)
{
	uint8 buffer[3*sizeof(uint8)];
	SbgErrorCode error;

	//
	// Build our arguments buffer
	//
	*buffer = permanent;
	*(buffer + sizeof(uint8)) = (uint8)contMode;
	*(buffer + 2*sizeof(uint8)) = divider;

	//
	// Send the command used to enable/disable the continuous mode
	//
	error = sbgProtocolSend(handle, SBG_SET_CONTINUOUS_MODE, buffer, 3*sizeof(uint8));

	if (error == SBG_NO_ERROR)
	{
		//
		// We should receive an ACK
		//
		error = sbgWaitForAck(handle, SBG_FRAME_RECEPTION_TIME_OUT);
	}

	return error;
}

/*!
 *	Returns our continous mode option.
 *	\param[in]	handle							A valid sbgCom library handle.
 *	\param[out]	pContMode						Returns the continuous mode
 *	\param[out]	pDivider						The current devider used to define the speed of our continous mode.
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgGetContinuousMode(SbgProtocolHandle handle, SbgContOutputTypes *pContMode, uint8 *pDivider)
{
	uint8 cmd;
	uint16 size;
	SbgErrorCode error;
	uint8 continuous[2];

	//
	// Send the command used to get continuous mode state
	//
	error = sbgProtocolSend(handle, SBG_GET_CONTINUOUS_MODE, NULL, 0);

	if (error == SBG_NO_ERROR)
	{
		//
		// Try to read our answer
		//
		error = sbgProtocolReceiveTimeOut(handle, &cmd, &continuous, &size, 2*sizeof(uint8));

		if (error == SBG_NO_ERROR)
		{
			if ( (cmd == SBG_RET_CONTINUOUS_MODE) && (size == 2*sizeof(uint8)) )
			{
				//
				// Returns our flags
				//
				if (pContMode)
				{
					*pContMode = (SbgContOutputTypes)(continuous[0]);
				}
				if (pDivider)
				{
					*pDivider = continuous[1];
				}

				return SBG_NO_ERROR;
			}
			else if ( (cmd == SBG_ACK) && (size == sizeof(uint8)) )
			{
				//
				// Get the error code contained in the ACK frame
				//
				error = (SbgErrorCode)(continuous[0]);

				//
				// We should receive an NACK so check if it's the case and return the error
				//
				if (error == SBG_NO_ERROR)
				{
					//
					// We have received an invalid frame, maybe a question/answer desync!
					//
					error = SBG_INVALID_FRAME;
				}
			}
			else
			{
				error = SBG_INVALID_FRAME;
			}
		}
	}

	return error;
}

/*!
 *	Define our default output mask used to enable outputs for the continous mode.
 *	\param[in]	handle							A valid sbgCom library handle.
 *	\param[in]	permanent						If true, stores this settings in flash memory.
 *	\param[in]	defaultOutputMask				Outputs to enable using a mask system.<br>
 *												See the file protocolOutput.h for a description of all available masks.
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgSetDefaultOutputMask(SbgProtocolHandle handle, bool permanent, uint32 defaultOutputMask)
{
	uint8 buffer[sizeof(uint8)+sizeof(uint32)];
	SbgErrorCode error;

	//
	// Build our arguments buffer
	//
	*buffer = permanent;
	*(uint32*)(buffer+sizeof(uint8)) = sbgHostToTarget32(handle->targetOutputMode, defaultOutputMask);

	//
	// Send the command used to set the default output mask
	//
	error = sbgProtocolSend(handle, SBG_SET_DEFAULT_OUTPUT_MASK, buffer, sizeof(uint8)+sizeof(uint32));

	if (error == SBG_NO_ERROR)
	{
		//
		// Try to receive an ACK
		//
		error = sbgWaitForAck(handle, SBG_FRAME_RECEPTION_TIME_OUT);

		//
		// Check if we have received an ACK
		//
		if (error == SBG_NO_ERROR)
		{
			//
			// Our new default output mask has been processed so update our protocol instance default output mask
			//
			handle->targetDefaultOutputMask = defaultOutputMask;
		}
	}

	return error;
}

/*!
 *	Returns the current default output mask used by the command SBG_GET_DEFAULT_OUTPUT.
 *	\param[in]	handle							A valid sbgCom library handle.
 *	\param[out]	pDefaultOutputMask				The current output mask used by our device.<br>
 *												See the file protocolOutput.h for a description of all available masks.
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgGetDefaultOutputMask(SbgProtocolHandle handle, uint32 *pDefaultOutputMask)
{
	uint8 cmd;
	uint16 size;
	uint32 defaultOutputMask;
	SbgErrorCode error;

	//
	// Send the command used to get th edefault output mask
	//
	error = sbgProtocolSend(handle, SBG_GET_DEFAULT_OUTPUT_MASK, NULL, 0);

	if (error == SBG_NO_ERROR)
	{
		//
		// Try to read our answer
		//
		error = sbgProtocolReceiveTimeOut(handle, &cmd, &defaultOutputMask, &size, sizeof(uint32));

		if (error == SBG_NO_ERROR)
		{
			if ( (cmd == SBG_RET_DEFAULT_OUTPUT_MASK) && (size == sizeof(uint32)) )
			{
				//
				// Define our default output mask for our current protocol instance
				//
				handle->targetDefaultOutputMask =sbgTargetToHost32(handle->targetOutputMode, defaultOutputMask);
			
				//
				// Returns our default output mask
				//
				if (pDefaultOutputMask)
				{
					*pDefaultOutputMask = handle->targetDefaultOutputMask;
				}	
			}
			else
			{
				error = SBG_INVALID_FRAME;
			}
		}
	}

	return error;
}

/*!
 *	Define triggered output options
 *	\param[in]	handle							A valid sbgCom library handle.
 *	\param[in]	permanent						If true, stores this settings in flash memory.
 *	\param[in]	condId							The condition number may be 0 to 3
 *	\param[in]	triggerMask						The trigger bit mask that will generate the triggered output
 *  \param[in]	outputMask						Output mask defining the output buffer sent when a trigger is detected
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgSetTriggeredMode(SbgProtocolHandle handle, bool permanent, uint8 condId, uint32 triggerMask, uint32 outputMask)
{
	uint8 buffer[2*sizeof(uint8)+2*sizeof(uint32)];
	SbgErrorCode error;

	//
	// Build our arguments buffer
	//
	buffer[0] = permanent;
	buffer[1] = condId;
	*(uint32*)(buffer+2*sizeof(uint8))					= sbgHostToTarget32(handle->targetOutputMode, triggerMask);
	*(uint32*)(buffer+2*sizeof(uint8)+sizeof(uint32))	= sbgHostToTarget32(handle->targetOutputMode, outputMask);

	//
	// Send the command used to set the default output mask
	//
	error = sbgProtocolSend(handle, SBG_SET_TRIGGERED_OUTPUT, buffer, 2*sizeof(uint8)+2*sizeof(uint32));

	if (error == SBG_NO_ERROR)
	{
		//
		// Try to receive an ACK
		//
		error = sbgWaitForAck(handle, SBG_FRAME_RECEPTION_TIME_OUT);
	}

	return error;
}

/*!
 *	Return a triggered output condition parameters
 *	\param[in]	handle							A valid sbgCom library handle.
 *	\param[in]	condId							The condition number may be 0 to 3
 *	\param[out]	pTriggerMask					The trigger bit mask that will generate the triggered output
 *  \param[out]	pOutputMask						Output mask defining the output buffer sent when a trigger is detected
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgGetTriggeredMode(SbgProtocolHandle handle, uint8 condId, uint32 *pTriggerMask, uint32 *pOutputMask)
{
	uint8 cmd;
	uint16 size;
	uint8 receiveBuffer[sizeof(uint8)+2*sizeof(uint32)];
	SbgErrorCode error;

	//
	// Send the command used to get the condId trigger channel options
	//
	error = sbgProtocolSend(handle, SBG_GET_TRIGGERED_OUTPUT, &condId, sizeof(uint8));

	if (error == SBG_NO_ERROR)
	{
		//
		// Try to read our answer
		//
		error = sbgProtocolReceiveTimeOut(handle, &cmd, receiveBuffer, &size, sizeof(uint8)+2*sizeof(uint32) );

		if (error == SBG_NO_ERROR)
		{
			if ( (cmd == SBG_RET_TRIGGERED_OUTPUT) && (size == sizeof(uint8)+2*sizeof(uint32)) )
			{
				//
				// And give the device answer to user
				//
				if (pTriggerMask)
				{
					*pTriggerMask = sbgTargetToHost32(handle->targetOutputMode, *(uint32*)(receiveBuffer+sizeof(uint8)));
				}
				if (pOutputMask)
				{
					*pOutputMask = sbgTargetToHost32(handle->targetOutputMode, *(uint32*)(receiveBuffer+sizeof(uint8)+sizeof(uint32)));
				}	
			}
			else
			{
				error = SBG_INVALID_FRAME;
			}
		}
	}

	return error;
}

//----------------------------------------------------------------------//
//- Applications commands operations                                   -//
//----------------------------------------------------------------------//

/*!
 *	Ask the device to returns the default ouput configured using sbgSetDefaultOutputMask.
 *	\param[in]	handle							A valid sbgCom library handle.
 *	\param[out]	pOutput							The returned output filled by the device according to the default output mask.
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgGetDefaultOutput(SbgProtocolHandle handle, SbgOutput *pOutput)
{
	uint8 cmd;
	uint16 size;
	uint8 receivedBuffer[SBG_MAX_DATA_LENGTH];
	SbgErrorCode error;

	//
	// Send the command used to get the default output
	//
	error = sbgProtocolSend(handle, SBG_GET_DEFAULT_OUTPUT, NULL, 0);

	if (error == SBG_NO_ERROR)
	{
		//
		// Try to read our answer
		//
		error = sbgProtocolReceiveTimeOut(handle, &cmd, receivedBuffer, &size, SBG_MAX_DATA_LENGTH);

		if (error == SBG_NO_ERROR)
		{
			if (cmd == SBG_RET_DEFAULT_OUTPUT)
			{
				//
				// Returns our output and fill our output structure
				//
				error = sbgFillOutputFromBuffer(handle->targetOutputMode, handle->targetDefaultOutputMask, receivedBuffer, size, pOutput);
			}
			else
			{
				error = SBG_INVALID_FRAME;
			}
		}
	}

	return error;
}

/*!
 *	Ask the device to returns a specific combinaison of ouputs.
 *	\param[in]	handle							A valid sbgCom library handle.
 *	\param[in]	outputMask						Using a mask system, define which outputs should be returned.<br>
 *												See the file protocolOutput.h for a description of all available masks.
 *	\param[out]	pOutput							The returned output filled by the device according to the default output mask.
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgGetSpecificOutput(SbgProtocolHandle handle, uint32 outputMask, SbgOutput *pOutput)
{
	uint8 cmd;
	uint16 size;
	uint8 receivedBuffer[SBG_MAX_DATA_LENGTH];
	SbgErrorCode error;
	uint32 targetOutputMask;

	//
	// Convert our value for our target
	//
	targetOutputMask = sbgHostToTarget32(handle->targetOutputMode, outputMask);

	//
	// Send the command used to get a specific output
	//
	error = sbgProtocolSend(handle, SBG_GET_SPECIFIC_OUTPUT, &targetOutputMask, sizeof(uint32));

	if (error == SBG_NO_ERROR)
	{
		//
		// Try to read our answer
		//
		error = sbgProtocolReceiveTimeOut(handle, &cmd, receivedBuffer, &size, SBG_MAX_DATA_LENGTH);

		if (error == SBG_NO_ERROR)
		{
			if (cmd == SBG_RET_SPECIFIC_OUTPUT)
			{
				//
				// Returns our output and fill our output structure
				//
				error = sbgFillOutputFromBuffer(handle->targetOutputMode, outputMask, receivedBuffer, size, pOutput);
			}
			else
			{
				error = SBG_INVALID_FRAME;
			}
		}
	}

	return error;
}
