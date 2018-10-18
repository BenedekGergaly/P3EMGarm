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

#include <Arduino.h>

#define TX_BUFFER_LEN 30
#define RX_BUFFER_LEN 30

#define DXL_LENGTH_POS 5
#define DXL_INSTRUCTION_POS 7

uint8_t tx_buffer[TX_BUFFER_LEN];
uint8_t rx_buffer[RX_BUFFER_LEN];

class servo
{
    public:
        void putInt8t(int8_t value, uint8_t* buffer, size_t pos);
        int8_t getInt8t(uint8_t* buffer, size_t pos);
        void putUint8t(uint8_t value, uint8_t* buffer, size_t pos);
        uint8_t getUint8t(uint8_t* buffer, size_t pos);
        void putInt16t(int16_t value, uint8_t* buffer, size_t pos);
        void putUint16t(uint16_t value, uint8_t* buffer, size_t pos);
        int16_t getInt16t(uint8_t* buffer, size_t pos);
        uint16_t getUint16t(uint8_t* buffer, size_t pos);
        void putInt32t(int32_t val, uint8_t* buffer, size_t pos);
        int32_t getInt32t(uint8_t* buffer, size_t pos);
        inline size_t getPackageLength(uint8_t* buffer);
        uint16_t calculateCrc(uint16_t crc_accum, uint8_t* data_blk_ptr, size_t data_blk_size);
        void addCrc(uint8_t* buffer, size_t data_size);
        boolean checkCrc(uint8_t* buffer, int16_t pos);
        void setHdrAndID(uint8_t id);
        void dumpPackage(uint8_t *buffer);
        void transmitPackage();
        bool receivePackage(size_t timeout = 100);
        void sendReadInstruction(uint8_t id, uint16_t from_addr, uint16_t data_length);
        void sendWriteInstruction(uint8_t id, uint16_t address, uint8_t data);
        void sendWriteInstruction(uint8_t id, uint16_t address, int32_t data);
        void sendWriteInstruction(uint8_t id, uint16_t address, int16_t data);
        bool doPing(uint8_t id, size_t timeout = 100);
        bool readPosition(uint8_t id, int32_t& position);
        bool setGoalPosition(uint8_t id, int32_t position);
        bool torqueEnable(uint8_t id, bool enable);
        bool setOperatingMode(uint8_t id, uint8_t mode);
        bool setGoalCurrent(uint8_t id, int16_t data);



        const uint16_t  crc_table[];
};
