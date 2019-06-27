#include <Arduino.h>
#include "DataStorage.h"

bool CaltionStorage::GetStorage(char version[], strCalStorage* out) {

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

bool CaltionStorage::PutStorage(char version[]) {

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

	}
}

CaltionStorage::CaltionStorage()
{
	memset(&_Storage, 0, sizeof(strCalStorage));
	_eeAddress = 0;
}

CaltionStorage::~CaltionStorage()
{
}
