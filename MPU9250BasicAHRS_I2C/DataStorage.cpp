#include <Arduino.h>
#include "DataStorage.h"

bool CalStorage::GetStorage(char version[], strCalStorage* out) {

	int elenght = EEPROM.length();
  
	if (elenght == sizeof(strCalStorage))
	{
		EEPROM.get(_eeAddress, _Storage);
		for (int i = 0; i < 5; i++) {
			if (_Storage.version[i] != version[i])
				return false;
		}
		memcpy(out, &_Storage, sizeof(strCalStorage));
		return true;
	}
	else {
		return false;
	}
}

bool CalStorage::PutStorage(char version[]) {

	int elenght = EEPROM.length();
	bool write = true;

	if (elenght == sizeof(strCalStorage))
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

CalStorage::CalStorage()
{
	memset(&_Storage, 0, sizeof(strCalStorage));
	_eeAddress = 0;
}

CalStorage::~CalStorage()
{
}
