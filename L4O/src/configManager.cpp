#include "configManager.h"
#include <cmath>
#include <cstring>
#include <cstdio>
#include <cstdarg>


Config::Config(ConfigSaver* cfg...)
{
	std::va_list args;
	va_start(args, cfg);
	configSavers.push_back(cfg);
	configSize += configSavers[0]->settingsSize;
	va_end(args);

}


// called whenever a config setting is changed to schedule a save after waiting to see if any more changes are being made
void Config::ScheduleSave()
{
	scheduleSave = true;
	saveBooked = SysTickVal;
}


// Write config settings to Flash memory
bool Config::SaveConfig(bool eraseOnly)
{
	scheduleSave = false;
	bool result = true;

	// FIXME - buffer size should be aligned to block of 64 bits
	uint8_t* flashConfig = reinterpret_cast<uint8_t*>(flashConfigAddr);

	// Check for config start and version number
	ConfigHeader* configHeader = (ConfigHeader*)flashConfig;

	uint32_t configBufferSize = configSize;
	bool addHeader = false;

	// Check if config header is valid: if so get position of current config block, else create header
	uint8_t* flashPos = flashConfig;
	if (strcmp(configHeader->startText, "CH") == 0 && configHeader->version == configVersion) {
		// Header is valid - locate next available location to store config at (first 1 in the bit array)
		flashPos += sizeof(ConfigHeader);
		uint32_t pos = 0;
		while (pos < 64 && configHeader->usedBitArray[pos++] == 0) {
			flashPos += configSize;
		}
	} else {
		addHeader = true;
		configBufferSize += sizeof(ConfigHeader);
	}

	uint8_t configBuffer[configBufferSize];					// Will hold all the data to be written to the

	// Config page does not start with header - create and add to write buffer
	uint32_t configPos = 0;
	if (addHeader) {
		auto ch = new (configBuffer) ConfigHeader{};		// use placement new to construct the header configBuffer
		configPos = sizeof(ConfigHeader);
	}

	// Add individual config settings to buffer
	for (auto& saver : configSavers) {
		memcpy(&configBuffer[configPos], saver->settingsAddress, saver->settingsSize);
		configPos += saver->settingsSize;
	}

	__disable_irq();										// Disable Interrupts
	FlashUnlock();											// Unlock Flash memory for writing
	FLASH->SR = FLASH_ALL_ERRORS;							// Clear error flags in Status Register

	// Check if flash needs erasing
	bool flashErased = false;
	for (uint32_t i = 0; i < configBufferSize / 4; ++i) {
		if (flashPos[i] != 0xFFFFFFFF) {
			FlashErasePage(flashConfigPage - 1);			// Erase page
			flashErased = true;
			break;
		}
	}

	// If the flash header is valid but the data region to be written to is not blank then the routine will need to be rerun to recreate the header
	if (!eraseOnly && !(flashErased && !addHeader)) {
		result = FlashProgram((uint32_t*)flashPos, reinterpret_cast<uint32_t*>(&configBuffer), configBufferSize);
	}

	FlashLock();											// Lock Flash
	__enable_irq(); 										// Enable Interrupts

	if (!eraseOnly && flashErased && !addHeader) {			// region to be written is not blank - rerun routine having erased page
		result = SaveConfig(false);
	}

	printf(result ? "Config Saved\r\n" : "Error saving config\r\n");

	return result;
}


void Config::SetConfig(uint8_t* configBuffer)
{
	// Serialise config values into buffer
//	strncpy(reinterpret_cast<char*>(configBuffer), "CFG", 4);		// Header
//	configBuffer[4] = configVersion;
//	uint32_t configPos = 8;											// Position in buffer to store data

	uint32_t configPos = 0;											// Position in buffer to store data
	for (auto& saver : configSavers) {
		memcpy(&configBuffer[configPos], saver->settingsAddress, saver->settingsSize);
		configPos += saver->settingsSize;
	}

	// Footer
	//strncpy(reinterpret_cast<char*>(&configBuffer[configPos]), "END", 4);
}


// Restore configuration settings from flash memory
void Config::RestoreConfig()
{
	uint8_t* flashConfig = reinterpret_cast<uint8_t*>(flashConfigAddr);

	// Check for config start and version number
	if (strcmp((char*)flashConfig, "CFG") == 0 && flashConfig[4] == configVersion) {
		uint32_t configPos = 8;											// Position in buffer to store data

		// Restore settings
		for (auto saver : configSavers) {
			memcpy(saver->settingsAddress, &flashConfig[configPos], saver->settingsSize);
			if (saver->validateSettings != nullptr) {
				saver->validateSettings();
			}
			configPos += saver->settingsSize;
		}

	}
}


// Unlock the FLASH control register access
void Config::FlashUnlock()
{
	if ((FLASH->CR & FLASH_CR_LOCK) != 0)  {
		FLASH->KEYR = 0x45670123U;					// These magic numbers unlock the flash for programming
		FLASH->KEYR = 0xCDEF89ABU;
	}
}


// Lock the FLASH Registers access
void Config::FlashLock()
{
	FLASH->CR |= FLASH_CR_LOCK;
}


void Config::FlashErasePage(uint8_t page)
{
	FLASH->CR &= ~FLASH_CR_PNB_Msk;
	FLASH->CR |= page << FLASH_CR_PNB_Pos;
	FLASH->CR |= FLASH_CR_PER;
	FLASH->CR |= FLASH_CR_STRT;
	FlashWaitForLastOperation();
	FLASH->CR &= ~FLASH_CR_PER;		// Unless this bit is cleared programming flash later throws a Programming Sequence error
}


bool Config::FlashWaitForLastOperation()
{
	if (FLASH->SR & FLASH_ALL_ERRORS) {						// If any error occurred abort
		FLASH->SR = FLASH_ALL_ERRORS;						// Clear all errors
		return false;
	}

	while ((FLASH->SR & FLASH_SR_BSY) == FLASH_SR_BSY) {

	}

	if ((FLASH->SR & FLASH_SR_EOP) == FLASH_SR_EOP) {		// Check End of Operation flag
		FLASH->SR = FLASH_SR_EOP;							// Clear FLASH End of Operation pending bit
	}

	return true;
}


bool Config::FlashProgram(uint32_t* dest_addr, uint32_t* src_addr, size_t size)
{
	// - requires that 64 bit words are written in rows of 32 bits
	if (FlashWaitForLastOperation()) {
		FLASH->CR |= FLASH_CR_PG;

		__ISB();
		__DSB();

		// Each write block is 64 bits
		for (uint16_t b = 0; b < std::ceil(static_cast<float>(size) / 8); ++b) {
			for (uint8_t i = 0; i < 2; ++i) {
				*dest_addr = *src_addr;
				++dest_addr;
				++src_addr;
			}

			if (!FlashWaitForLastOperation()) {
				FLASH->CR &= ~FLASH_CR_PG;				// Clear programming flag
				return false;
			}
		}

		__ISB();
		__DSB();

		FLASH->CR &= ~FLASH_CR_PG;						// Clear programming flag
	}
	return true;
}


