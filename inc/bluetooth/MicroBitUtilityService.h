/*
The MIT License (MIT)

This class has been written by Sam Kent for the Microbit Educational Foundation.

Permission is hereby granted, free of charge, to any person obtaining a
copy of this software and associated documentation files (the "Software"),
to deal in the Software without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, and to permit persons to whom the
Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
DEALINGS IN THE SOFTWARE.
*/

#ifndef MICROBIT_UTILITY_SERVICE_H
#define MICROBIT_UTILITY_SERVICE_H

#include "MicroBitConfig.h"

#if CONFIG_ENABLED(DEVICE_BLE)

#include "MicroBitBLEManager.h"
#include "MicroBitBLEService.h"
#include "MicroBitMemoryMap.h"

#include "MicroBitFlash.h"
#include "MicroBitStorage.h"

#include "MicroBitComponent.h"
#include "MicroBitEvent.h"
#include "EventModel.h"
#include "MicroBitLog.h"


#define MICROBIT_ID_UTILITY_PROCESS 0


/**
  * Class definition for the custom MicroBit Partial Flash Service.
  * Provides a BLE service to remotely read the memory map and flash the PXT program.
  */
class MicroBitUtilityService : public MicroBitBLEService
{
    static MicroBitUtilityService *shared;
    
    public:
    /**
     * Return a pointer to the shared service
     * @return a pointer to the service, or NULL if it has not been created
     */
    static MicroBitUtilityService *getShared()
    {
        return shared;
    }

    /**
     * Return a pointer to the shared service, creating it if necessary
     * @param _ble an instance of BLEDevice
     * @param _messageBus An instance of a MessageBus to interface with.
     * @param _storage A persistent storage manager to use to hold non-volatile state.
     * @param _log a log storage manager to read stored data.
     * @return a pointer to the service, or NULL if it has not been created
     */
    static MicroBitUtilityService *createShared( BLEDevice &_ble, EventModel &_messageBus, MicroBitStorage &_storage, MicroBitLog &_log);

    /**
     * Destructor for the Utility Service.
     */
    virtual ~MicroBitUtilityService();

    public:
    /**
     * Constructor.
     * @param _ble an instance of BLEDevice
     * @param _messageBus An instance of a MessageBus to interface with.
     * @param _storage A persistent storage manager to use to hold non-volatile state.
     * @param _log a log storage manager to read stored data.
     */
    MicroBitUtilityService( BLEDevice &_ble, EventModel &_messageBus, MicroBitStorage &_storage, MicroBitLog &_log);
    
    /**
     * Invoked when BLE connects.
    */
    void onConnect( const microbit_ble_evt_t *p_ble_evt);


    /**
     * Invoked when BLE disconnects.
    */
    void onDisconnect( const microbit_ble_evt_t *p_ble_evt);

    /**
     * Callback. Invoked when any of our attributes are written via BLE.
     */
    void onDataWritten(const microbit_ble_evt_write_t *params);

    /**
     * Callback. Invoked when a registered event occurs.
     */
    void onEvent(MicroBitEvent e);

    private:
    // MessageBus we're using
    EventModel          &messageBus;
    MicroBitStorage     &storage;
    MicroBitLog         &log;

    // Ensure packets are in order
    uint8_t packetCount = 0;
    uint8_t blockPacketCount = 0;

    uint8_t characteristicValue[ 20];

    // Index for each charactersitic in arrays of handles and UUIDs
    typedef enum mbbs_cIdx
    {
        mbbs_cIdxCTRL,
        mbbs_cIdxCOUNT
    } mbbs_cIdx;
    
    // UUIDs for our service and characteristics
    static const uint16_t serviceUUID;
    static const uint16_t charUUID[ mbbs_cIdxCOUNT];
    
    // Data for each characteristic when they are held by Soft Device.
    MicroBitBLEChar      chars[ mbbs_cIdxCOUNT];

    public:
    int              characteristicCount()          { return mbbs_cIdxCOUNT; };
    MicroBitBLEChar *characteristicPtr( int idx)    { return &chars[ idx]; };
    
    private:
    
    typedef struct request_t
    {
        uint8_t  job;       // Client cycles high nibble i.e. { 0x00, 0x10, 0x20, ..., 0xF0, 0x00, ...}
        uint8_t  type;
        uint8_t  data[18];
    } request_t;
    
    typedef struct reply_t
    {
        uint8_t  job;       // Service cycles low nibble i.e client job + { 0x00, 0x01, 0x02, ..., 0x0E, 0x00, ... }
        uint8_t  data[19];  // low nibble == 0x0F indicates error
    } reply_t;
    
    typedef struct requestLog_t
    {
        uint8_t  job;
        uint8_t  type;
        uint8_t  format;
    } requestLog_t;

    typedef struct requestLogRead_t
    {
        uint8_t  job;
        uint8_t  type;
        uint8_t  format;
        uint8_t  reserved;
        uint32_t index;
        uint32_t length;
    } requestLogRead_t;

    typedef enum type_t
    {
        typeLogNone,
        typeLogLength, // reply data =  { uint32_t length }
        typeLogRead    // reply data = up to 19 bytes
    } type_t;

    typedef struct workspace_t
    {
        request_t request;
        reply_t   reply;
        int8_t    state;
        uint8_t   length;
        uint8_t   jobLow;
    } workspace_t;

    workspace_t *workspace;
    
    static const uint8_t jobLowMAX = 0x0E;
    static const uint8_t jobLowERR = 0x0F;  // reply data = { int32_t error }

    /**
     * Allocate workspace
     * @return pointer to workspace on success or NULL
     */
    workspace_t *workspaceAlloc();

    /**
     * Initialise workspace
     */
    void workspaceInit();

    /**
     * Free workspace
     */
    void workspaceFree();

    /**
     * Send a reply packet
     */
    int sendReply( int idx, const reply_t *data, uint16_t length);

    /**
     * Process the current request
     */
    void processRequest();

    /**
     * Process request typeLogLength
     */
    int processLogLength();

    /**
     * Process request typeLogRead
     */
    int processLogRead();
};


#endif
#endif
