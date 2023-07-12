#pragma once

#include "initialisation.h"
//#include "USB.h"


static uint32_t* const addrFlashPage16 = reinterpret_cast<uint32_t*>(0x08007800);			 // Base address of page 16, 2 Kbytes
#define FLASH_ALL_ERRORS (FLASH_SR_OPERR  | FLASH_SR_PROGERR | FLASH_SR_WRPERR | FLASH_SR_PGAERR | FLASH_SR_SIZERR | FLASH_SR_PGSERR | FLASH_SR_MISERR | FLASH_SR_FASTERR | FLASH_SR_RDERR  | FLASH_SR_OPTVERR)

class Config {
public:
	static constexpr uint32_t configVersion = 1;
	static constexpr uint32_t BufferSize = 100;
	static constexpr bool eraseConfig = true;

	bool scheduleSave = false;
	uint32_t saveBooked;
	uint8_t configBuffer[BufferSize];	// Need a large buffer as drum sequence data is ~6k

	void Calibrate();
	void ScheduleSave();				// called whenever a config setting is changed to schedule a save after waiting to see if any more changes are being made
	bool SaveConfig(bool eraseOnly = false);
	uint32_t SetConfig();				// Serialise configuration data into buffer
	void RestoreConfig();				// gets config from Flash, checks and updates settings accordingly

	void FlashUnlock();
	void FlashLock();
	void FlashErasePage(uint8_t Sector);
	bool FlashWaitForLastOperation();
	bool FlashProgram(uint32_t* dest_addr, uint32_t* src_addr, size_t size);
private:


};

extern Config configManager;
