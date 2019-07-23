/*
  Aero-Quad Carballo-Rinke - May 2019 -

  Copyright (c) 2019 Mario Leal.  All rights reserved.
  An Open Source Arduino based multicopter.

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef _DATA_STORAGE_H_
#define _DATA_STORAGE_H_
#include <EEPROM.h>
// Utilities for writing and reading from the EEPROM

#define STORAGE_VERSION	"VER01"

typedef struct strCalStorage {
	char version[5];
	uint8_t gyroStorage[6];
	uint8_t acelStorage[6];
	uint8_t magnStorage[6];
};


class CalStorage
{
	public:
		bool GetStorage(char version[], strCalStorage* out);
		bool PutStorage(char version[]);
	public:
		CalStorage();
		~CalStorage();

	private:
		int _eeAddress;
		strCalStorage _Storage;
		char _version[5 + 1];
};



#endif //_DATA_STORAGE_H_

