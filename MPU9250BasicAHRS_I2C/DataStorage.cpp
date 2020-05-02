#include <Arduino.h>
#include "DataStorage.h"

bool EEStorage::GetStorage(char version[], strCaltionStorage* out) {

	int elenght = EEPROM.length();
  
	if (elenght == sizeof(strCaltionStorage))
	{
		EEPROM.get(_eeAddress, _Storage);
		for (int i = 0; i < 5; i++) {
			if (_Storage.version[i] != version[i])
				return false;
		}
		memcpy(out, &_Storage, sizeof(strCaltionStorage));
		return true;
	}
	else {
		return false;
	}
}

bool EEStorage::PutStorage(char version[]) {

	int elenght = EEPROM.length();
	bool write = true;

	if (elenght == sizeof(strCaltionStorage))
	{
		EEPROM.get(_eeAddress, _Storage);
		for (int i = 0; i < 5; i++) {
			if (_Storage.version[i] != version[i]) {
				write |= true;
			}
			else {
				write |= false;
			}
		}
	}

	if (write) {
		EEPROM.put(_eeAddress, _Storage);
	}
}

EEStorage::EEStorage()
{
	memset(&_Storage, 0, sizeof(strCaltionStorage));
	_eeAddress = 0;
}

EEStorage::~EEStorage()
{
}
