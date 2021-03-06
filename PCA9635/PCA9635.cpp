#include "PCA9635.h"

// Constructor
pca9635::pca9635()
{
    device_address = 0x70; // Default to the all-call address
    autoincrement_bits = 0x80; // Autoincrement all
}

void pca9635::begin(byte dev_addr, boolean wire_begin, boolean init)
{
    i2c_device::begin(dev_addr, wire_begin);
    if (init)
    {
        this->set_sleep(0x0); // Wake up oscillators
        delayMicroseconds(500); // Wait for the oscillators to stabilize
        this->set_led_mode(3); // Default to full PWM mode for all drivers
    }
}

void pca9635::getTest() {
	uint8_t d[32]= { };
	i2c_device::read_many(0x80,0x1C,d);
	for(uint8_t i=0; i<0x1C; i++){
		//Serial.print("R:");
		if(i<16) Serial.print("0");
		Serial.print((i),HEX);
		Serial.print("=");
		if(d[i]<16) Serial.print("0");
		Serial.print(d[i],HEX);
		Serial.print(" ");
	}
	Serial.println();
}

void pca9635::setMode(uint8_t mode1, uint8_t mode2) {
	uint8_t Data[2]={ 0,0 };	
	Data[0]=mode1;
	Data[1]=mode2;
	i2c_device::write_many(0x80, 2, Data);	
	pca9635::setDefault();
}

void pca9635::setDefault() {
	uint8_t Data[10]={ 0xFF, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xE2, 0xE4, 0xE8, 0xE0 };	
	i2c_device::write_many(0x92, 10, Data);	
}

void pca9635::begin(byte dev_addr, boolean wire_begin)
{
    pca9635::begin(dev_addr, wire_begin, false);
}

/**
 * Control the led channel mode, modes: 
 * 0=fully off
 * 1=fully on (no PWM)
 * 2=individual PWM only
 * 3=individual and group PWM
 *
 * Remember that led numbers start from 0
 */
boolean pca9635::set_led_mode(byte ledno, byte mode)
{
    byte reg = 0x17;
    if (   8 < ledno
        && ledno < 11)
    {
        reg = 0x16;
    }
    if (   4 < ledno
        && ledno < 8)
    {
        reg = 0x15;
    }
    if (ledno < 4)
    {
        reg = 0x14;
    }
    byte value = 0;
    switch (mode)
    {
        case 0:
            value = B00000000;
            break;
        case 1:
            value = B01010101;
            break;
        case 2:
            value = B10101010;
            break;
        case 3:
            value = B11111111;
            break;
    }
    byte mask = B00000000;
    switch(ledno%4)
    {
        case 0:
            mask = (byte)~B00000011;
            break;
        case 1:
            mask = (byte)~B00001100;
            break;
        case 2:
            mask = (byte)~B00110000;
            break;
        case 3:
            mask = (byte)~B11000000;
            break;
    }
    return this->read_modify_write(reg | autoincrement_bits, mask, value);
}

/**
 * Set mode for all leds 
 * 0=fully off
 * 1=fully on (no PWM)
 * 2=individual PWM only
 * 3=individual and group PWM
 */
boolean pca9635::set_led_mode(byte mode)
{
    byte value;
    switch (mode)
    {
        case 0:
            value = B00000000;
            break;
        case 1:
            value = B01010101;
            break;
        case 2:
            value = B10101010;
            break;
        case 3:
            value = B11111111;
            break;
    }
    byte values[] = { value, value, value, value };
    return this->write_many(0x14 | autoincrement_bits, 4, values);
}

/**
 * Enable given SUBADDRess (1-3)
 */
boolean pca9635::enable_subaddr(byte addr)
{
    byte value;
    switch (addr)
    {
        case 1:
            value = _BV(3); // 0x71
            break;
        case 2:
            value = _BV(2); // 0x72
            break;
        case 3:
            value = _BV(1); // 0x74
            break;
    }
    byte mask = ~value;
    return this->read_modify_write(0x0 | autoincrement_bits, mask, value);
}

/**
 * Changes the driver mdoe between open drain(0x0) and totem-pole (0xff, the default)
 */
boolean pca9635::set_driver_mode(byte mode)
{
    return this->read_modify_write(0x01 | autoincrement_bits, (byte)~_BV(4), mode << 4);
}

/**
 * Changes the oscillator mode between sleep (0xff, the default) and active (0x0)
 */
boolean pca9635::set_sleep(byte sleep)
{
    return this->read_modify_write(0x00 | autoincrement_bits, (byte)~_BV(4), sleep << 4);
}


/**
 * Sets the pwm value for given led, note that it must have previously been enabled for PWM control with set_mode
 * 
 * Remember that led numbers start from 0
 */
boolean pca9635::set_led_pwm(byte ledno, byte cycle)
{
    byte reg = 0x02 + ledno;
    return this->write_many(reg | autoincrement_bits, 1, &cycle);
}


/**
 * Do the software-reset song-and-dance, this should reset all drivers on the bus
 */
boolean pca9635::reset()
{
#ifdef I2C_DEVICE_DEBUG
    Serial.println(F("pca9635::reset() called"));
#endif
    byte result = I2c.write(0x03, 0xa5, 0x5a);
    if (result > 0)
    {
#ifdef I2C_DEVICE_DEBUG
        Serial.print(F("FAILED: I2c.write(0x03, 0x5a, 0xa5); returned: "));
        Serial.println(result, DEC);
#endif
        return false;
    }
    delayMicroseconds(5); // Wait for the reset to complete
    return true;
}



// Instance for the all-call address
pca9635 PCA9635 = pca9635();
