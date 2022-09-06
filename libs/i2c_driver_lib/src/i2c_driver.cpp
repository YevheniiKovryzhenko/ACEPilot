/*
 * i2c_driver.cpp
 *
 * Author:	Yevhenii Kovryzhenko, Department of Aerospace Engineering, Auburn University.
 * Contact: yzk0058@auburn.edu
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED ''AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL I
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * Last Edit:  08/26/2022 (MM/DD/YYYY)
 */

#include "i2c_driver.hpp"

 // preposessor macros
#ifndef unlikely
#define unlikely(x)	__builtin_expect (!!(x), 0)
#endif // !unlikely

#ifndef likely
#define likely(x)	__builtin_expect (!!(x), 1)
#endif // !likely

int i2c::open(int new_driver_bus_id)
{
	driver_bus_id = new_driver_bus_id;
	set_devAddr(DEF_I2C_ADDRESS);

	if (unlikely(rc_i2c_init(driver_bus_id, devAddr) == -1))
	{
		printf("ERROR: Failed to configure i2c bus with id: %d and devAddr: %d\n", driver_bus_id, devAddr);
		initialized = 0;
		return -1;
	}
	initialized = 1;
	return 0;
}

int i2c::open(int new_driver_bus_id, uint8_t new_devAddr)
{
	driver_bus_id = new_driver_bus_id;
	set_devAddr(new_devAddr);

	if (unlikely(rc_i2c_init(driver_bus_id, devAddr) == -1))
	{
		printf("ERROR: Failed to configure i2c bus with id: %d and devAddr: %d\n", driver_bus_id, devAddr);
		return -1;
	}
	initialized = 1;
	return 0;
}

int i2c::close(void)
{
	if (likely(initialized == 1))
	{
		if (unlikely(rc_i2c_close(driver_bus_id) == -1))
		{
			printf("\nERROR: Failed to close i2c port %d", driver_bus_id);
			return -1;
		}
	}
	else
	{
		printf("WARNING: i2c was not initialized for servo driver, can't close\n");
	}
	initialized = 0;
	return 0;
}

int i2c::change_address(uint8_t devAddr)
{
	if (likely(check_init()))
	{
		int dscr = rc_i2c_set_device_address(driver_bus_id, devAddr);
		if (unlikely(dscr == -1))
		{
			printf("ERROR: failed to change i2c address to %d for device id: %d\n", devAddr, driver_bus_id);
			return -1;
		}
		return dscr;
	}
	else
	{
		return -1;
	}
}

int i2c::read_byte(uint8_t regAddr, uint8_t* data)
{
	if (likely(check_init()))
	{
		if (unlikely(rc_i2c_read_byte(driver_bus_id, regAddr, data) == -1))
		{
			printf("ERROR: failed to read byte from i2c register %d for device id: %d\n", regAddr, driver_bus_id);
			return -1;
		}
	}
	else
	{
		return -1;
	}
	return 0;
}

int i2c::read_bytes(uint8_t regAddr, size_t count, uint8_t* data)
{
	if (likely(check_init()))
	{
		if (unlikely(rc_i2c_read_bytes(driver_bus_id, regAddr, count, data) == -1))
		{
			printf("ERROR: failed to read bytes from i2c register %d for device id: %d\n", regAddr, driver_bus_id);
			return -1;
		}
	}
	else
	{
		return -1;
	}
	return 0;
}

int i2c::read_word(uint8_t regAddr, uint16_t* data)
{
	if (likely(check_init()))
	{
		if (unlikely(rc_i2c_read_word(driver_bus_id, regAddr, data) == -1))
		{
			printf("ERROR: failed to read word from i2c register %d for device id: %d\n", regAddr, driver_bus_id);
			return -1;
		}
	}
	else
	{
		return -1;
	}
	return 0;
}

int i2c::read_words(uint8_t regAddr, size_t count, uint16_t* data)
{
	if (likely(check_init()))
	{
		if (unlikely(rc_i2c_read_words(driver_bus_id, regAddr, count, data) == -1))
		{
			printf("ERROR: failed to read words from i2c register %d for device id: %d\n", regAddr, driver_bus_id);
			return -1;
		}
	}
	else
	{
		return -1;
	}
	return 0;
}

int i2c::write_byte(uint8_t regAddr, uint8_t data)
{
	if (likely(check_init()))
	{
		if (unlikely(rc_i2c_write_byte(driver_bus_id, regAddr, data) == -1))
		{
			printf("ERROR: failed to write byte to i2c register %d for device id: %d\n", regAddr, driver_bus_id);
			return -1;
		}
	}
	else
	{
		return -1;
	}
	return 0;
}

int i2c::write_bytes(uint8_t regAddr, size_t count, uint8_t* data)
{
	if (likely(check_init()))
	{
		if (unlikely(rc_i2c_write_bytes(driver_bus_id, regAddr, count, data) == -1))
		{
			printf("ERROR: failed to write bytes to i2c register %d for device id: %d\n", regAddr, driver_bus_id);
			return -1;
		}
	}
	else
	{
		return -1;
	}
	return 0;
}

int i2c::write_word(uint8_t regAddr, uint16_t data)
{
	if (likely(check_init()))
	{
		if (unlikely(rc_i2c_write_word(driver_bus_id, regAddr, data) == -1))
		{
			printf("ERROR: failed to write word to i2c register %d for device id: %d\n", regAddr, driver_bus_id);
			return -1;
		}
	}
	else
	{
		return -1;
	}
	return 0;
}

int i2c::write_words(uint8_t regAddr, size_t count, uint16_t* data)
{
	if (likely(check_init()))
	{
		if (unlikely(rc_i2c_write_words(driver_bus_id, regAddr, count, data) == -1))
		{
			printf("ERROR: failed to write words to i2c register %d for device id: %d\n", regAddr, driver_bus_id);
			return -1;
		}
	}
	else
	{
		return -1;
	}
	return 0;
}

int i2c::send_bytes(size_t count, uint8_t* data)
{
	if (likely(check_init()))
	{
		if (unlikely(rc_i2c_send_bytes(driver_bus_id, count, data) == -1))
		{
			printf("ERROR: failed to send bytes to i2c to device id: %d\n", driver_bus_id);
			return -1;
		}
	}
	else
	{
		return -1;
	}
	return 0;
}

int i2c::send_byte(uint8_t data)
{
	if (likely(check_init()))
	{
		if (unlikely(rc_i2c_send_byte(driver_bus_id, data) == -1))
		{
			printf("ERROR: failed to send byte to i2c to device id: %d\n", driver_bus_id);
			return -1;
		}
	}
	else
	{
		return -1;
	}
	return 0;
}

int i2c::send_devAddr(void)
{
	if (likely(check_init()))
	{
		if (unlikely(send_byte(devAddr) == -1))
		{
			printf("ERROR: failed to send device address %d, hex:%x\n",devAddr,devAddr);
			return -1;
		}
	}
	else
	{
		return -1;
	}
	return 0;
}

int i2c::lock_bus(void)
{
	if (likely(check_init()))
	{
		if (unlikely(rc_i2c_lock_bus(driver_bus_id) == -1))
		{
			printf("ERROR: failed to lock bus with id: %d\n", driver_bus_id);
			return -1;
		}
	}
	else
	{
		return -1;
	}
	return 0;
}

void i2c::set_devAddr(uint8_t new_devAddr)
{
	devAddr = new_devAddr;
	return;
}

int i2c::unlock_bus(void)
{
	if (likely(check_init()))
	{
		if (unlikely(rc_i2c_unlock_bus(driver_bus_id) == -1))
		{
			printf("ERROR: failed to unlock bus with id: %d\n", driver_bus_id);
			return -1;
		}
	}
	else
	{
		return -1;
	}
	return 0;
}

int i2c::get_lock(void)
{
	if (likely(check_init()))
	{
		if (unlikely(rc_i2c_get_lock(driver_bus_id) == -1))
		{
			printf("ERROR: failed to get lock for the bus with id: %d\n", driver_bus_id);
			return -1;
		}
	}
	else
	{
		return -1;
	}
	return 0;
}

int i2c::get_fd(void)
{
	if (likely(check_init()))
	{
		if (unlikely(rc_i2c_get_fd(driver_bus_id) == -1))
		{
			printf("ERROR: failed to get file descriptor for the bus with id: %d\n", driver_bus_id);
			return -1;
		}
	}
	else
	{
		return -1;
	}
	return 0;
}

bool i2c::check_init(void)
{
	if (likely(initialized == 1))
	{
		return 1;
	}
	else
	{
		printf("ERROR: i2c not initialized, call open() first.\n");
		return 0;
	}
}