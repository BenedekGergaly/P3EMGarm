/*
 * Copyright (c) 2018 Jens Dalsgaard Nielsen
 * Copyright (c) 2018 Karl Damkjær Hansen
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "servo.h"

servo::servo()
{
    
}



 // Low-level functions for setting individual bytes in the buffers.
void servo::putInt8t(int8_t value, uint8_t* buffer, size_t pos)
{
  buffer[pos] = value;
}

int8_t servo::getInt8t(uint8_t* buffer, size_t pos)
{
  return buffer[pos];
}

void servo::putUint8t(uint8_t value, uint8_t* buffer, size_t pos)
{
  buffer[pos] = value;
}

uint8_t servo::getUint8t(uint8_t* buffer, size_t pos)
{
  return buffer[pos];
}

void servo::putInt16t(int16_t value, uint8_t* buffer, size_t pos)
{
  buffer[pos] = (uint8_t)(value & 0x00ff);
  buffer[pos + 1] = (uint8_t)(value >> 8);
}

void servo::putUint16t(uint16_t value, uint8_t* buffer, size_t pos)
{
  buffer[pos] = (uint8_t)(value & 0x00ff);
  buffer[pos + 1] = (uint8_t)(value >> 8);
}

int16_t servo::getInt16t(uint8_t* buffer, size_t pos)
{
  int16_t v = 0;
  v = buffer[pos + 1];
  v = v << 8;
  v = v | buffer[pos];
  return v;
}

uint16_t servo::getUint16t(uint8_t* buffer, size_t pos)
{
  uint16_t v = 0;
  v = buffer[pos + 1];
  v = v << 8;
  v = v | buffer[pos];
  return v;
}

void servo::putInt32t(int32_t val, uint8_t* buffer, size_t pos)
{
  for (int16_t i = 0; i < 4 ; i++) {
    buffer[pos + i] = (uint8_t)(val & 0x000000ff);
    val = val >> 8;
  }
}

int32_t servo::getInt32t(uint8_t* buffer, size_t pos)
{
  int32_t v = 0;
  for (int16_t i = 3; i > -1 ; --i) {
    v = v << 8;
    v = v | (int32_t)buffer[pos + i];
  }
  return v;
}


// Dynamixel Protocol 2.0
// The protocol defines a header with fixed positions for the instruction and
// length fields:
#define DXL_LENGTH_POS 5
#define DXL_INSTRUCTION_POS 7

inline size_t servo::getPackageLength(uint8_t* buffer)
{
  return getUint16t(buffer, DXL_LENGTH_POS);
}

// Cyclic Redundancy Check (CRC)
// The protocol uses CRC to check for faults in the transmissions, see:
// https://en.wikipedia.org/wiki/Cyclic_redundancy_check. Specifically
// Dynamixel uses a variant of the CRC16 algorithm used in MODBUS also called
// CRC-16-IBM, see http://emanual.robotis.com/docs/en/dxl/crc/.


uint16_t servo::calculateCrc(uint16_t crc_accum, uint8_t* data_blk_ptr, size_t data_blk_size)
{
  size_t i, j;

  for (j = 0; j < data_blk_size; j++)
  {
    i = ((uint16_t)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
#ifdef ARDUINOMEMUSE
    crc_accum = (crc_accum << 8) ^  pgm_read_word_near(crc_table + i);
#else
    crc_accum = (crc_accum << 8) ^ crc_table[i];
#endif
  }
  return crc_accum;
}

// Add CRC to a buffer
// Calculates the CRC and appends it to the buffer.
//
// @param buffer    The buffer to calculate CRC for.
// @param data_size The number of bytes in the buffer to calculate the CRC for.
//                  Incidentally, this is also the position in the array where
//                  the CRC will be added.
void servo::addCrc(uint8_t* buffer, size_t data_size)
{
  uint16_t crc;
  crc = calculateCrc(0, buffer, data_size);
  putInt16t(crc, tx_buffer, data_size);
}

// Check the CRC of a package
// Calculates the CRC of the data in the buffer and compares it to the received
// CRC checksum.
//
// @param buffer the buffer holding the package.
// @param pos the position of the first CRC byte in the buffer.
// @return true if the CRC check is successful.
boolean servo::checkCrc(uint8_t* buffer, int16_t pos)
{
  uint16_t incomming_crc = getUint16t(buffer, pos);
  uint16_t calculated_crc = calculateCrc(0, buffer, pos);
  return (calculated_crc == incomming_crc);
}

void servo::setHdrAndID(uint8_t id)
{
  tx_buffer[0] = 0xff;
  tx_buffer[1] = 0xff;
  tx_buffer[2] = 0xfd;
  tx_buffer[3] = 0x00;
  tx_buffer[4] = id;
}


void servo::dumpPackage(uint8_t *buffer)
{
  size_t l = getPackageLength(buffer) + 7;
  for (size_t i = 0; i < l; i++) {
    Serial.print((int)buffer[i], HEX); Serial.print(" ");
  }
  Serial.println("");
}

// Transmit the package in the tx_buffer
void servo::transmitPackage()
{
  size_t pgk_length = getPackageLength(tx_buffer) + 7;
  Serial1.write(tx_buffer, pgk_length);
}

// Try to receive a package
//
// @param timeout milliseconds to wait for a reply.
// @returns true if a package was received and the CRC checks out.
bool servo::receivePackage(size_t timeout)
{
  elapsedMillis since_start = 0;
  size_t bytecount = 0;
  size_t remaining_read = 1;
  while (remaining_read > 0 && since_start < timeout)
  {
    if (Serial1.available())
    {
      uint8_t incomming_byte = Serial1.read();
      switch (bytecount)
      {
        case 0:
        case 1: if (incomming_byte == 0xFF) {
                  rx_buffer[bytecount] = incomming_byte;
                  ++bytecount;
                } else {
                  bytecount = 0;
                }
                break;
        case 2: if (incomming_byte == 0xFD)
                {
                  rx_buffer[bytecount] = incomming_byte;
                  ++bytecount;
                } else {
                  bytecount = 0;
                }
                break;
        case 3:
        case 4:
        case 5: rx_buffer[bytecount] = incomming_byte;
                ++bytecount;
                break;
        case 6: rx_buffer[bytecount] = incomming_byte;
                remaining_read = getPackageLength(rx_buffer);
                ++bytecount;
                break;
        default: rx_buffer[bytecount] = incomming_byte;
                 ++bytecount;
                 --remaining_read;
                 break;
      }
    }
  }
  if (remaining_read == 0)
  {
    //dumpPackage(rx_buffer, getPackageLength(rx_buffer));
    return checkCrc(rx_buffer, bytecount-2);
  }
  else
  {
    return false;
  }
}

// Send a READ instruction to the servo
void servo::sendReadInstruction(uint8_t id, uint16_t from_addr, uint16_t data_length)
{
  /* Read instruction package
  0     1     2     3     4    5      6     7     8      9       10      11      12    13
  H1    H2    H3    RSRV  ID   LEN1   LEN2  INST  PARAM1 PARAM2  PARAM3  PARAM4  CRC1  CRC2
  0xFF  0xFF  0xFD  0x00  0x01  0x07  0x00  0x02  0x84   0x00    0x04    0x00    0x1D  0x15
  */
  setHdrAndID(id);
  putUint16t(7, tx_buffer, DXL_LENGTH_POS);
  putUint8t(0x02, tx_buffer, DXL_INSTRUCTION_POS);
  putUint16t(from_addr, tx_buffer, 8);
  putUint16t(data_length, tx_buffer, 10);
  addCrc(tx_buffer, 12);

  transmitPackage();
}

// Send a 1 byte READ instruction to the servo
void servo::sendWriteInstruction(uint8_t id, uint16_t address, uint8_t data)
{
  setHdrAndID(id);
  putUint16t(6, tx_buffer, DXL_LENGTH_POS);
  putUint8t(0x03, tx_buffer, DXL_INSTRUCTION_POS);
  putUint16t(address, tx_buffer, 8);
  putUint8t(data, tx_buffer, 10);
  addCrc(tx_buffer, 11);

  transmitPackage();
}

// Send a 4 byte READ instruction to the servo
void servo::sendWriteInstruction(uint8_t id, uint16_t address, int32_t data)
{
  /* Read instruction package
  0    1    2    3    4    5    6    7    8    9    10   11   12   13   14   15
  H1   H2   H3   RSRV ID   LEN1 LEN2 INST P1   P2   P3   P4   P5   P6   CRC1 CRC2
  0xFF 0xFF 0xFD 0x00 0x01 0x09 0x00 0x03 0x74 0x00 0x00 0x02 0x00 0x00 0xCA 0x89
  */
  setHdrAndID(id);
  putUint16t(9, tx_buffer, DXL_LENGTH_POS);
  putUint8t(0x03, tx_buffer, DXL_INSTRUCTION_POS);
  putUint16t(address, tx_buffer, 8);
  putInt32t(data, tx_buffer, 10);
  addCrc(tx_buffer, 14);

  transmitPackage();
}

//2 byte packet
void servo::sendWriteInstruction(uint8_t id, uint16_t address, int16_t data)
{
  setHdrAndID(id);
  putUint16t(7, tx_buffer, DXL_LENGTH_POS);
  putUint8t(0x03, tx_buffer, DXL_INSTRUCTION_POS);
  putUint16t(address, tx_buffer, 8);
  putUint16t(data, tx_buffer, 10);
  addCrc(tx_buffer, 12);

  transmitPackage(); 
}


// Ping a servo
// Pings the servo and waits for a response. If none is received within the
// timeout, the ping fails. There is no check as to whether the response is
// appropriate. When sending a ping to broadcast (0xFE), several packages may be
// received. The user should look into the received package to see if the result
// is satisfactory.
//
// @param  id      the servo ID to ping.
// @param  timeout milliseconds before timing out.
// @return true if response is received.
bool servo::doPing(uint8_t id, size_t timeout)
{
  /* Ping Instruction Packet
    0     1     2     3     4     5     6     7     8     9
    H1    H2    H3    RSRV  ID    LEN1  LEN2  INST  CRC1  CRC2
    0xFF  0xFF  0xFD  0x00  0x01  0x03  0x00  0x01  0x19  0x4E
  */
  setHdrAndID(id);
  putUint16t(3, tx_buffer, DXL_LENGTH_POS); // length is at pos 5 and 6
  putUint8t(0x01, tx_buffer, DXL_INSTRUCTION_POS); // ping instruction (0x01) at pos 7
  addCrc(tx_buffer, 8); // CRC at pos 8 and 9
  transmitPackage();

  bool package_received = receivePackage(timeout);

  return package_received;
}

// Read the position of a servo
//
// @param id       the servo ID to set the position of.
// @param position the position to go to.
// @return true if the servo responded.
bool servo::readPosition(uint8_t id, int32_t& position)
{
  sendReadInstruction(id, 132, 4);
  if (receivePackage(100))
  {
    position = getInt32t(rx_buffer, 9);
    return true;
  }
  else
  {
    return false;
  }
}

// Set the goal position of a servo
// Reads the position of a servo, if no response is read, the function does not
// modify the value of the position variable.
//
// @param id the servo ID to get the position of.
// @param position a reference to store the read position in.
// @return true if the servo responded.
bool servo::setGoalPosition(uint8_t id, int32_t position)
{
  sendWriteInstruction(id, 116, position);
  if (receivePackage(100))
  {
    return true;
  }
  else
  {
    return false;
  }
}

// Enable torque on the servo
// The torque must be enabled for the motor to move.
//
// @param id     the servo ID to set the position of.
// @param enable whether the torque should be enabled or not.
// @return true if the servo responded.
bool servo::torqueEnable(uint8_t id, bool enable)
{
  uint8_t enable_data = 0x01;
  if (! enable)
  {
    enable_data = 0x00;
  }
  sendWriteInstruction(id, 64, enable_data);
  if (receivePackage(100))
  {
    return true;
  }
  else
  {
    return false;
  }
}

//0 = current (torque)
//3 = position
bool servo::setOperatingMode(uint8_t id, uint8_t mode)
{
    sendWriteInstruction(id, 11, mode);
    if(receivePackage(100))
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool servo::setGoalCurrent(uint8_t id, int16_t data)
{
    sendWriteInstruction(id, 102, data);
    if(receivePackage(100))
    {
        return true;
    }
    else
    {
        return false;
    }
}
