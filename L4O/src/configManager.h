#pragma once

#include "initialisation.h"
#include  <bitset>


static uint32_t* const addrFlashPage16 = reinterpret_cast<uint32_t*>(0x08007800);			 // Base address of page 16, 2 Kbytes
#define FLASH_ALL_ERRORS (FLASH_SR_OPERR  | FLASH_SR_PROGERR | FLASH_SR_WRPERR | FLASH_SR_PGAERR | FLASH_SR_SIZERR | FLASH_SR_PGSERR | FLASH_SR_MISERR | FLASH_SR_FASTERR | FLASH_SR_RDERR  | FLASH_SR_OPTVERR)

// Struct added to classes that need settings saved
struct ConfigSaver {
	void* settingsAddress;
	uint32_t settingsSize;
	void (*validateSettings)(void);
};


class Config {
public:
	static constexpr uint32_t configVersion = 1;
	static constexpr uint32_t BufferSize = 100;

	// STM32G431 is Category 2 (manual p74). Flash : 16 pages * 2k = 32k
	static constexpr uint32_t flashConfigPage = 16;
	static constexpr uint32_t flashPageSize = 2048;
	uint32_t* const flashConfigAddr = reinterpret_cast<uint32_t* const>(0x08000000 + flashPageSize * (flashConfigPage - 1));

	bool scheduleSave = false;
	uint32_t saveBooked = false;

	std::vector<ConfigSaver*> configSavers;
	uint32_t configSize = 0;

	Config(ConfigSaver* cfg...);
	void Calibrate();
	void ScheduleSave();				// called whenever a config setting is changed to schedule a save after waiting to see if any more changes are being made
	bool SaveConfig(bool eraseOnly = false);
	void SetConfig(uint8_t* configBuffer);				// Serialise configuration data into buffer
	void RestoreConfig();				// gets config from Flash, checks and updates settings accordingly

private:
	struct ConfigHeader {
		char startText[2] = {'C', 'H'};
		uint16_t version = configVersion;
		std::bitset<64> usedBitArray {0xFFFFFFFF'FFFFFFFF};
	};

	void FlashUnlock();
	void FlashLock();
	void FlashErasePage(uint8_t Sector);
	bool FlashWaitForLastOperation();
	bool FlashProgram(uint32_t* dest_addr, uint32_t* src_addr, size_t size);


};

extern Config config;
