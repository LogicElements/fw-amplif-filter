/**
  * @file       comm_proto.h
  * @version    $(APP_VERSION)
  * @date       $(RELEASE_DATE)
  * @brief      file_brief
  * @author     jan.bartovsky
  *
  * @copyright  Logic Elements Copyright
  *
  * @defgroup gr group_name
  * @{
  * @brief group_brief
  *
  * This module contains
  *
  * @par Main features:
  *
  * @par Example
  * @code
  *
  * @endcode
  */
#ifndef COM_PROTO_H_
#define COM_PROTO_H_

/* Includes ------------------------------------------------------------------*/

#include "common.h"

//#include "event_mngr.h"

/* Definitions----------------------------------------------------------------*/

#define COM_PROTO_HEADER_LEN          4

#define COM_PROTO_UART_START_BYTE     0x90

#define COM_PROTO_MIN_LENGTH          8


typedef enum
{
  COM_PROTO_ERROR          = 1,    ///< Program Error code
  COM_PROTO_READ_REG_RESP  = 5,    ///< Response to ReadRegister command
  COM_PROTO_TIME_SER       = 8,    ///< Time series
  COM_PROTO_EVENT_RESP     = 12,   ///< Response to Event request
  COM_PROTO_FW_UPG_ACK     = 127,  ///< Acknowledge of FW upgrade packet
  COM_PROTO_WRITE_REG      = 129,  ///< Write register request
  COM_PROTO_READ_REG       = 130,  ///< Read register request
  COM_PROTO_GET_EVENT      = 131,  ///< Get events from local buffer
  COM_PROTO_FW_UPGRADE     = 255,  ///< Read configuration from external flash memory
}ComProto_PacketId_t;



/* Typedefs-------------------------------------------------------------------*/

/* Functions -----------------------------------------------------------------*/

/**
 * Initialize module (nothing at the moment)
 * @return Status
 */
Status_t ComProto_Init(void);

/**
 * Get length of received packet
 * @param data Pointer to packet
 * @return Length of packet in bytes
 */
uint16_t ComProto_GetLength(uint8_t *data);

/**
 * Check the length and basic consistency of received packet
 * @param data Pointer to packet
 * @param length Length of received data
 * @return STATUS_OK if we have received the entire packet
 */
Status_t ComProto_CheckProtoLength(uint8_t *data, uint16_t length);

/**
 * Process the received packet
 *
 * The received packet is processed according to the type of packet (read, write, fw_upgr)
 * and the response is created.
 * @param data Received packet to process
 * @param txResp Response buffer
 * @return Status
 */
Status_t ComProto_ProcessPacket(uint8_t *data, uint8_t* txResp);

/**
 * Fill data series packet header in the buffer
 * @param buffer Pointer to buffer that will accommodate data series packet
 * @param id Identifier of data series
 * @param timestamp Timestamp of data series
 * @param delta Delta of two consecutive samples
 * @param count Number of samples to follow
 * @return Status
 */
Status_t ComProto_FillSeries(uint8_t *buffer, uint32_t id, uint32_t timestamp, uint32_t delta, uint32_t count);




#endif /* COM_PROTO_H_ */
/** @} */
