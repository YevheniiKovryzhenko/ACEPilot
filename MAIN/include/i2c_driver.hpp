/*
 * i2c_driver.hpp
 *
 * Copyright Yevhenii Kovryzhenko, Department of Aerospace Engineering, Auburn University.
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
 */

/* Summary 
This is a simple C++ wrap of i2c robotcontrol library as C++ class
http://strawsondesign.com/docs/librobotcontrol/group___i2_c.html
includes simple error printing
*/

#ifndef I2C_DRIVER_HPP
#define I2C_DRIVER_HPP
#include <rc/i2c.h>
#include <stdio.h>


#define DEF_I2C_ADDRESS 0x40      /**< Default I2C Address */

/*!
 *  @brief  Class that stores state and functions for interacting with i2c drivers
 */
class i2c {
private:
	int initialized = 0;
	uint8_t devAddr = DEF_I2C_ADDRESS;
	int driver_bus_id = 0;
	bool check_init(void);
public:
	/* Initializes a bus and sets it to talk to a particular device address. */
	/* Returns 0 on success or -1 on failure */
	int open(int new_driver_bus_id);
	int open(int new_driver_bus_id, uint8_t devAddr);
	
	
	/* Closes an I2C bus. */
	/* Returns 0 on success or -1 on failure */
	int close(void);


	/* Changes the device address the bus is configured to talk to.
	Actually changing the device address in the I2C driver requires a system call 
	and is relatively slow. This function records which device address the bus 
	is set to and will only make the system call if the requested address is 
	different than the set address. This makes it safe to call this function 
	repeatedly with no performance penalty. */
	/* Returns { description_of_the_return_value } */
	int change_address(uint8_t devAddr);


	/* Reads a single byte from a device register.
	This sends the device address and register address to be read from 
	before reading the response, works for most i2c devices. */
	/* Returns 0 on success or -1 on failure */
	int read_byte(uint8_t regAddr, uint8_t* data);


	/* Reads multiple bytes from a device register.
	This sends the device address and register address to be read from 
	before reading the response, works for most i2c devices. */
	/* Returns 0 on success or -1 on failure */
	int read_bytes(uint8_t regAddr, size_t count, uint8_t* data);


	/* Reads a single word (16 bits) from a device register.
	This sends the device address and register address to be read from 
	before reading the response, works for most i2c devices. */
	/* Returns 0 on success or -1 on failure */
	int read_word(uint8_t regAddr, uint16_t* data);


	/* Reads multiple words (16 bytes each) from a device register.
	This sends the device address and register address to be read from 
	before reading the response, works for most i2c devices. */
	/* Returns 0 on success or -1 on failure */
	int read_words(uint8_t regAddr, size_t count, uint16_t* data);


	/* Writes a single byte to a specified register address.
	This sends the device address and register address followed by 
	the actual data to be written. Works for most i2c devices. */
	/* Returns 0 on success or -1 on failure */
	int write_byte(uint8_t regAddr, uint8_t data);


	/* Writes a multiple bytes to a specified register address.
	This sends the device address and register address followed by
	the actual data to be written. Works for most i2c devices. */
	/* Returns 0 on success or -1 on failure */
	int write_bytes(uint8_t regAddr, size_t count, uint8_t* data);


	/* Writes a single word (16 bits) to a specified register address.
	This sends the device address and register address followed by 
	the actual data to be written. Works for most i2c devices. */
	/* Returns 0 on success or -1 on failure */
	int write_word(uint8_t regAddr, uint16_t data);


	/* Writes a multple words (16 bits each) to a specified register address.
	This sends the device address and register address followed by
	the actual data to be written. Works for most i2c devices. */
	/* Returns 0 on success or -1 on failure */
	int write_words(uint8_t regAddr, size_t count, uint16_t* data);


	/* Sends exactly user-defined data without prepending a register address.
	Instead of automatically sending a device address before the data which is 
	typical for reading/writing registers, the rc_i2c_send_bytes function send 
	only the data given by the data argument. This is useful for more complicated 
	IO such as uploading firmware to a device. */
	/* Returns 0 on success or -1 on failure */
	int send_bytes(size_t count, uint8_t* data);


	/* Sends exactly user-defined data without prepending a register address.
	Instead of automatically sending a device address before the data which is 
	typical for reading/writing registers, the rc_i2c_send_bytes function send 
	only the data given by the data argument. This is useful for more complicated 
	IO such as uploading firmware to a device. */
	/* Returns 0 on success or -1 on failure */
	int send_byte(uint8_t data);

	/* Sends just the register address and nothing else.*/
	int send_devAddr(void);

	/* Sets the internall devAddr var */
	void set_devAddr(uint8_t new_devAddr);


	/* Locks the bus so other threads in the process know the bus is in use.
	
	Locking a bus is similar to locking a mutex, it is a way for threads to 
	communicate within one process when sharing a bus. This, however, is not 
	a hard lock in the sense that it does not block and does not stop any of 
	the other functions in this API from being called. It only serves as a flag 
	that can be checked between threads if the user chooses to do so. This 
	is encouraged in multithraded applications to prevent timing-sensitive i2c 
	communication from being interrupted but is not enforced.
	
	All read/write functions in this API will lock the bus during the transaction 
	and return the lockstate to what it was at the beginning of the transaction. 
	Ideally the user should lock the bus themselves before a sequence of 
	transactions and unlock it afterwards. */
	/* Returns the lock state (0 or 1) when this function is called, or -1 on error. */
	int lock_bus(void);


	/* Unlocks a bus to indicate to other threads in the process that the bus is now free.
	see lock_bus for further description. */
	/* Returns the lock state (0 or 1) when this function is called, or -1 on error. */
	int unlock_bus(void);


	/* Fetches the current lock state of the bus. */
	/* Returns 0 if unlocked, 1 if locked, or -1 on error. */
	int get_lock(void);


	/* Gets file descriptor. */
	/* Returns file descriptor of the specified bus or -1 on failure */
	int get_fd(void);
};

#endif /* I2C_DRIVER_HPP */