/*
 * Copyright 2015-16 Hillcrest Laboratories, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License and 
 * any applicable agreements you may have with Hillcrest Laboratories, Inc.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Simple Utility functions common to several SH2 files.
 */

#include "sh2_util.h"

uint32_t readu32(const uint8_t *p)
{
	uint32_t retval = p[0] | (p[1] << 8) | (p[2] << 16) | (p[3] << 24);
	return retval;
}

uint16_t readu16(const uint8_t *p)
{
	uint32_t retval = p[0] | (p[1] << 8);
	return retval;
}

uint8_t readu8(const uint8_t *p)
{
	uint32_t retval = p[0];
	return retval;
}

uint32_t readNu32(const uint8_t *p, uint8_t len)
{
	if (len == 4) return readu32(p);
	if (len == 2) return readu16(p);
	if (len == 1) return readu8(p);
	return 0x00000000;
}

int32_t read32(const uint8_t *p)
{
	int32_t retval = p[0] | (p[1] << 8) | (p[2] << 16) | (p[3] << 24);
	return retval;
}

int16_t read16(const uint8_t *p)
{
	int32_t retval = p[0] | (p[1] << 8);
	return retval;
}

int8_t read8(const uint8_t *p)
{
	int32_t retval = p[0];
	return retval;
}
