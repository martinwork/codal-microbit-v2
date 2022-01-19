/*
The MIT License (MIT)

Copyright (c) 2016 British Broadcasting Corporation.
This software is provided by Lancaster University by arrangement with the BBC.

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

/**
  * Class definition for the custom MicroBit Partial Flashing service.
  * Provides a BLE service to remotely write the user program to the device.
  */
#include "MicroBitConfig.h"

#if CONFIG_ENABLED(DEVICE_BLE)

#include "MicroBitUtilityService.h"
#include "MicroBit.h"


using namespace codal;

const uint16_t MicroBitUtilityService::serviceUUID               = 0x0001;
const uint16_t MicroBitUtilityService::charUUID[ mbbs_cIdxCOUNT] = { 0x0002 };


MicroBitUtilityService *MicroBitUtilityService::shared = NULL;


/**
 * Return a pointer to the shared service, creating it if necessary
 * @param _ble an instance of BLEDevice
 * @param _messageBus An instance of a MessageBus to interface with.
 * @param _storage A persistent storage manager to use to hold non-volatile state.
 * @return a pointer to the service, or NULL if it has not been created
 */
MicroBitUtilityService *MicroBitUtilityService::createShared( BLEDevice &_ble, EventModel &_messageBus, MicroBitStorage &_storage, MicroBitLog &_log)
{
    if ( !shared)
        shared = new MicroBitUtilityService( _ble, _messageBus, _storage, _log);
    return shared;
}


/**
 * Constructor.
 * Create a representation of the Partial Flash Service
 * @param _ble The instance of a BLE device that we're running on.
 * @param _messageBus An instance of a MessageBus to interface with.
 * @param _storage A persistent storage manager to use to hold non-volatile state.
 */
MicroBitUtilityService::MicroBitUtilityService( BLEDevice &_ble, EventModel &_messageBus, MicroBitStorage &_storage, MicroBitLog &_log) :
    messageBus(_messageBus), storage(_storage), log(_log)
{
    // Initialise data
    memclr( characteristicValue, sizeof( characteristicValue));
    workspace = NULL;

    // Register the base UUID and create the service.
    RegisterBaseUUID( bs_base_uuid);
    CreateService( serviceUUID);

    CreateCharacteristic( mbbs_cIdxCTRL, charUUID[ mbbs_cIdxCTRL],
                         characteristicValue,
                         0, sizeof(characteristicValue),
                         microbit_propWRITE | microbit_propWRITE_WITHOUT | microbit_propNOTIFY);
}


/**
 * Destructor for the Utility Service.
 */
MicroBitUtilityService::~MicroBitUtilityService()
{
    workspaceFree();
    shared = NULL;
}


/**
  * Invoked when BLE connects.
  */
void MicroBitUtilityService::onConnect( const microbit_ble_evt_t *p_ble_evt)
{
    messageBus.listen( MICROBIT_ID_UTILITY, MICROBIT_EVT_ANY, this, &MicroBitUtilityService::onEvent);
}


/**
  * Invoked when BLE disconnects.
  */
void MicroBitUtilityService::onDisconnect( const microbit_ble_evt_t *p_ble_evt)
{
    messageBus.ignore( MICROBIT_ID_UTILITY, MICROBIT_EVT_ANY, this, &MicroBitUtilityService::onEvent);
    workspaceInit();
}


/**
 * Callback. Invoked when any of our attributes are written via BLE.
 */
void MicroBitUtilityService::onDataWritten(const microbit_ble_evt_write_t *params)
{
    if ( params->handle == valueHandle( mbbs_cIdxCTRL) && params->len > 0 && params->len < sizeof(request_t))
    {
        if ( workspaceAlloc())
        {
            memcpy( &workspace->request, params->data, params->len);
            MicroBitEvent evt( MICROBIT_ID_UTILITY, MICROBIT_ID_UTILITY_PROCESS);
        }
    }
}


/**
 * Callback. Invoked when a registered event occurs.
 */
void MicroBitUtilityService::onEvent(MicroBitEvent e)
{
    switch (e.source)
    {
        case MICROBIT_ID_UTILITY:
            switch(e.value)
            {
                case MICROBIT_ID_UTILITY_PROCESS:
                    processRequest();
                    break;
            }
            break;
    }
}


/**
 * Allocate workspace
 * @return pointer to workspace on success or NULL
 */
MicroBitUtilityService::workspace_t *MicroBitUtilityService::workspaceAlloc()
{
    if ( !workspace)
    {
        workspace = (workspace_t *) malloc( sizeof( workspace_t));
    }
    workspaceInit();
    return workspace;
}


/**
 * Initialise workspace
 */
void MicroBitUtilityService::workspaceInit()
{
    if ( workspace)
    {
        memclr( workspace, sizeof( workspace_t));
    }
}


/**
 * Free workspace
 */
void MicroBitUtilityService::workspaceFree()
{
    free( workspace);
    workspace = NULL;
}


/**
 * Send a reply packet
 */
int MicroBitUtilityService::sendReply( int idx, const reply_t *reply, uint16_t length)
{
    bool ok = notifyChrValue( idx, (const uint8_t *) reply, length);
    MICROBIT_DEBUG_DMESG("MicroBitUtilityService::sendReply %d %x %d = %d", idx, (unsigned long) reply, (int) length, ok ? 1 : 0);
    
    // On success, cycle jobLow for next reply packet
    if ( ok && ( reply->job & 0x0F) != jobLowERR)
    {
      workspace->jobLow++;
      if ( workspace->jobLow > jobLowMAX)
      {
        workspace->jobLow = 0;
      }
    }
    return ok;
}


/**
 * Process the current request
 */
void MicroBitUtilityService::processRequest()
{
    if ( !workspace)
        return;
    
    int finished = 0;
    
    switch ( workspace->request.type)
    {
        case typeLogLength:
            finished = processLogLength();
            break;
        case typeLogRead:
            finished = processLogRead();
            break;
        default:
            finished = 1;
            break;
    }
    
    if ( finished)
    {
        workspaceInit();
    }
    else
    {
        // TODO: create an event when BLE queue has space?
        MicroBitEvent evt( MICROBIT_ID_UTILITY, MICROBIT_ID_UTILITY_PROCESS);
    }
}


/**
 * Process request typeLogLength
 */
int MicroBitUtilityService::processLogLength()
{
    requestLog_t    *request = (requestLog_t *) &workspace->request;
    reply_t         *reply   = &workspace->reply;
    // workspace->state == 1 => reply has been prepared but not sent
    if ( workspace->state == 0)
    {
        workspace->state = 1;
        uint32_t length = log.getDataLength( (DataFormat) request->format);
        reply->job = request->job + workspace->jobLow;
        memcpy( reply->data, &length, sizeof(uint32_t));
        workspace->length = sizeof(uint32_t);
        MICROBIT_DEBUG_DMESG("MicroBitUtilityService::processLogLength %x", (unsigned int) length);
    }
    return sendReply( mbbs_cIdxCTRL, reply, 1 + workspace->length);
}


/**
 * Process request typeLogRead
 */
int MicroBitUtilityService::processLogRead()
{
    requestLogRead_t *request = (requestLogRead_t *) &workspace->request;
    reply_t          *reply   = &workspace->reply;
    
    while ( request->length)
    {
        if ( workspace->state == 0)
        {
            int block = min( request->length, 19);
            int result = log.readData( (DataFormat) request->format, request->index, reply->data, block);
            MICROBIT_DEBUG_DMESG("log.readData %d %d %d = %d", (int) request->format, (int) request->index, (int) block, (int) result);
            for ( int i = 0; i < block; i++)
              MICROBIT_DEBUG_DMESG("%x ", (unsigned int) reply->data[i]);
            if ( result)
            {
                workspace->state = 2;
                reply->job = request->job + jobLowERR;
                memcpy( reply->data, &result, sizeof(int));
                workspace->length = sizeof(int);
            }
            else
            {
                workspace->state = 1;
                reply->job = request->job + workspace->jobLow;
                workspace->length = block;
            }
        }
        
        if ( !sendReply( mbbs_cIdxCTRL, reply, offsetof(reply_t, data) + workspace->length))
        {
            return 0;
        }
        
        if ( workspace->state == 2)
        {
            return 1;
        }
        
        workspace->state = 0;
        request->index  += workspace->length;
        request->length -= workspace->length;
    }
    
    return 1;
}


#endif
