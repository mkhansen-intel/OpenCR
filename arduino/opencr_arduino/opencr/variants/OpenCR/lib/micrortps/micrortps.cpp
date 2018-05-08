
#include <stdlib.h>
#include <string.h>

#if 0

#include "micrortps.hpp"

#include "hw.h"


#define RTPS_BUFFER_SIZE     4096

//-- Internal Variables
//
typedef struct
{
  RtpsResponse_t create;
  RtpsResponse_t update;
  RtpsResponse_t delete;
  RtpsResponse_t lookup;
  RtpsResponse_t read;
  RtpsResponse_t write;
} RtpsResponseTable_t;

//-- Internal Functions
//
void on_status_received(XRCEInfo info, uint8_t operation, uint8_t status, void* args);
bool microRtpsCheckResponse(uint16_t object_id, RtpsProcess_t process, uint32_t timeout);


static bool is_rtps_init_done = false;
static ClientState *rtps_client;
static XRCEInfo     client_info;


void rclcpp::init(void)
{
  bool ret;

  if(rtps_client == NULL)
  {
    rtps_client = new_serial_client_state(RTPS_BUFFER_SIZE, "opencr_usb");
  }

  client_info = create_client(rtps_client, on_status_received, &ResponseTable);
  send_to_agent(rtps_client);

  ret = microRtpsCheckResponse(client_info.object_id, CREATE_CLIENT, 500);

  if(ret == true)
  {
    is_rtps_init_done = true;
  }
}

bool rclcpp::ok(void)
{
  return is_rtps_init_done;
}

void rclcpp::shutdown(void)
{

}

void rclcpp::spin()
{

}

void rclcpp::spin_some()
{

}





static bool microRtpsCheckResponse(uint16_t object_id, RtpsProcess_t process, uint32_t timeout)
{
  bool ret = false;
  uint32_t pre_time;

  pre_time = millis();
  while((millis() - pre_time < timeout))
  {
    receive_from_agent(rtps_client);

    switch(process)
    {
      case CREATE_CLIENT :
      case CREATE_PARTICIPANT:
      case CREATE_TOPIC:
      case CREATE_PUBLISHER:
      case CREATE_SUBSCRIBER:
      case CREATE_WRITER:
      case CREATE_READER:
        if(ResponseTable.create.ids.object_id == object_id
            && (ResponseTable.create.status == STATUS_OK
            || ResponseTable.create.status == STATUS_ERR_ALREADY_EXISTS))
        {
            ret = true;
        }
        break;
    }
  }
  return ret;
}

static void on_status_received(XRCEInfo info, uint8_t operation, uint8_t status, void* args)
{
  RtpsResponseTable_t *tbl = (RtpsResponseTable_t *) args;
  RtpsResponse_t *recv;

  if(tbl == NULL)
  {
    return;
  }

  switch(operation)
  {
    case STATUS_LAST_OP_CREATE:
      recv = (RtpsResponse_t *) &tbl->create;
      break;
    case STATUS_LAST_OP_UPDATE:
      recv = (RtpsResponse_t *) &tbl->update;
      break;
    case STATUS_LAST_OP_DELETE:
      recv = (RtpsResponse_t *) &tbl->delete;
      break;
    case STATUS_LAST_OP_LOOKUP:
      recv = (RtpsResponse_t *) &tbl->lookup;
      break;
    case STATUS_LAST_OP_READ:
      recv = (RtpsResponse_t *) &tbl->read;
      break;
    case STATUS_LAST_OP_WRITE:
      recv = (RtpsResponse_t *) &tbl->write;
      break;
    default :
      return;
  }

  recv->ids = info;
  recv->operation = operation;
  recv->status = status;
}

#endif
