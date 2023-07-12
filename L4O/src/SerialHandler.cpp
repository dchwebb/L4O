#include "SerialHandler.h"
#include "configManager.h"

#include <stdio.h>
#include <cmath>
#include "lfo.h"


SerialHandler::SerialHandler(USBHandler& usbObj)
{
	usb = &usbObj;

	// bind the usb's CDC caller to the CDC handler in this class
	usb->cdcDataHandler = std::bind(&SerialHandler::Handler, this, std::placeholders::_1, std::placeholders::_2);
}



// Check if a command has been received from USB, parse and action as required
bool SerialHandler::Command()
{
	char buf[50];

	if (!cmdPending) {
		return false;
	}

	if (cmd.compare("info\n") == 0) {		// Print diagnostic information

		usb->SendString("Mountjoy QuadEnv v1.0 - Current Settings:\r\n\r\n");

		// Use manual float conversion as printf with float support uses more space than we have
		uint32_t fltIntPart = std::round(lfos.config.durationMult);
		uint32_t fltFrcPart = std::round((lfos.config.durationMult - fltIntPart) * 100);
		sprintf(buf, "Envelope multiplier: %ld.%ld\r\n", fltIntPart, fltFrcPart);
		usb->SendString(buf);

	} else if (cmd.compare("help\n") == 0) {

		usb->SendString("Mountjoy QuadEnv\r\n"
				"\r\nSupported commands:\r\n"
				"info        -  Show diagnostic information\r\n"
				"mult:xx.x   -  Set duration multiplier (eg 1.0 for short, 8.5 for long)\r\n"
				"\r\n"
#if (USB_DEBUG)
				"usbdebug    -  Start USB debugging\r\n"
				"\r\n"
#endif
		);

#if (USB_DEBUG)
	} else if (cmd.compare("usbdebug\n") == 0) {				// Configure gate LED
		USBDebug = true;
		usb->SendString("Press link button to dump output\r\n");
#endif

	} else if (cmd.compare(0, 5, "mult:") == 0) {		// Set envelope duration multiplier
		const float mult = ParseFloat(cmd, ':', 0.1f, 99.9f);
		if (mult >= 0.0f) {
			lfos.config.durationMult = mult;
			configManager.SaveConfig();
		}

	} else {
		usb->SendString("Unrecognised command: " + cmd + "Type 'help' for supported commands\r\n");
	}

	cmdPending = false;
	return true;
}


void SerialHandler::Handler(uint8_t* data, uint32_t length)
{
	static bool newCmd = true;
	if (newCmd) {
		cmd = std::string(reinterpret_cast<char*>(data), length);
		newCmd = false;
	} else {
		cmd.append(reinterpret_cast<char*>(data), length);
	}
	if (*cmd.rbegin() == '\r')
		*cmd.rbegin() = '\n';

	if (*cmd.rbegin() == '\n') {
		cmdPending = true;
		newCmd = true;
	}

}

int32_t SerialHandler::ParseInt(const std::string cmd, const char precedingChar, int low = 0, int high = 0) {
	int32_t val = -1;
	int8_t pos = cmd.find(precedingChar);		// locate position of character preceding
	if (pos >= 0 && std::strspn(cmd.substr(pos + 1).c_str(), "0123456789-") > 0) {
		val = stoi(cmd.substr(pos + 1));
	}
	if (high > low && (val > high || val < low)) {
		usb->SendString("Must be a value between " + std::to_string(low) + " and " + std::to_string(high) + "\r\n");
		return low - 1;
	}
	return val;
}

float SerialHandler::ParseFloat(const std::string cmd, const char precedingChar, float low = 0.0, float high = 0.0) {
	float val = -1.0f;
	int8_t pos = cmd.find(precedingChar);		// locate position of character preceding
	if (pos >= 0 && std::strspn(cmd.substr(pos + 1).c_str(), "0123456789.") > 0) {
		val = stof(cmd.substr(pos + 1));
	}
	if (high > low && (val > high || val < low)) {
		usb->SendString("Must be a value between " + std::to_string(low) + " and " + std::to_string(high) + "\r\n");
		return low - 1.0f;
	}
	return val;
}
