/**
   Writes the current speed obtained by odometry
   and sends this to the Pixhawk via MAVLink

   Currently, this sends a dummy value and does not
   read current speed. This is for TESTING!
   Once this is confirmed to work, the next version will be written.
*/

// In case we need a second serial port for debugging
#define SOFT_SERIAL_DEBUGGING   // Comment this line if no serial debugging is needed
#ifdef SOFT_SERIAL_DEBUGGING
  // Library to use serial debugging with a second board
  #include <SoftwareSerial.h>
  SoftwareSerial mySerial(11, 12); // RX, TX // Connected to second Arduino feeding into computer
  SoftwareSerial pxSerial(9, 10);  // RX, TX // Pixhawk connected to these pins
#endif

#include "mavlink.h"
//#include "common/mavlink_msg_request_data_stream.h"

// Mavlink variables
unsigned long previousMillisMAVLink = 0;     // will store last time MAVLink was transmitted and listened
unsigned long next_interval_MAVLink = 1000;  // next interval to count
const int num_hbs = 60;                      // # of heartbeats to wait before activating STREAMS from Pixhawk. 60 = one minute.
int hbs_count = num_hbs;                     // # of heartbeats counted

unsigned long previousMillis = 0;     // will store last time LED was updated
unsigned long next_interval = 0;      // next interval

void setup() {
  // TODO - changed setup because it had Serial.begin() when not using those pins
  //    and mySerial() didn't have a begin statement (but we don't use this)
  // TODO - make the entire system use the SOFT_SERIAL_DEBUGGING param, otherwise what we do
  // doesn't make much sense. This file isn't consistent.
  
  // Starting Serial communication for print statements
  Serial.begin(57600);

  #ifdef SOFT_SERIAL_DEBUGGING
    // [DEB] Soft serial port start
    pxSerial.begin(57600);
    pxSerial.println("MAVLink starting.");
//    mySerial.begin(57600);
  #endif
}

void loop() {
  // TODO - the old code managed the lights first
  unsigned long currentMillis = millis();
  
  // TODO: next_interval is just control flow for when to manage lights

  // Normal mode, lights on.
  if (currentMillis - previousMillis >= next_interval) {
    // Keep time last mode changed
    previousMillis = currentMillis;
  }

  // MAVLink /////////////////////////////////////

  // The default UART header for your MCU
  int sysid = 1;                    // 1 PX, 255 ground station. ID 20 for an airplane, in an example
  int compid = 0  ;                 // The component sending the message
  int type = MAV_TYPE_GROUND_ROVER; // This system is a rover

  // Define the system type, in this case an airplane -> on-board controller
  // uint8_t system_type = MAV_TYPE_FIXED_WING;
  uint8_t system_type = MAV_TYPE_GENERIC;         // TODO - try GROUND_ROVER
  uint8_t autopilot_type = MAV_AUTOPILOT_INVALID; // TODO - ???

  uint8_t system_mode = MAV_MODE_PREFLIGHT; // Booting up
  uint32_t custom_mode = 0;                 // Custom mode, can be defined by user/adopter
  uint8_t system_state = MAV_STATE_STANDBY; // System ready for flight
  
  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
  //mavlink_msg_heartbeat_pack(sysid,compid, &msg, type, autopilot_type, system_mode, custom_mode, system_state);
  mavlink_msg_heartbeat_pack(1, 0, &msg, type, autopilot_type, system_mode, custom_mode, system_state);

  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  // Send the message with the standard UART send function
  // (uart0_send might be named differently depending on
  // the individual microcontroller / library in use)
  unsigned long currentMillisMAVLink = millis();
  if (currentMillisMAVLink - previousMillisMAVLink >= next_interval_MAVLink) {
    // We save the last time the mode was changed
    previousMillisMAVLink = currentMillisMAVLink;

    // Actually send the heartbeat message
    #ifdef SOFT_SERIAL_DEBUGGING
      pxSerial.write(buf, len);
      //mySerial.println("Arduino Heartbeat");
    #else
      Serial.write(buf, len);
    #endif

    // Increment the counter, since we sent a heartbeat/request
    hbs_count++;

    // Request some information if we've seen a lot of heartbeats (even minute, as it's currently set up)
    if (hbs_count >= num_hbs) {
      #ifdef SOFT_SERIAL_DEBUGGING
        Serial.println("Mav_Request_Data() called!");
      #endif

      // Request streams from Pixhawk and reset the counter
      Mav_Request_Data();
      hbs_count = 0;
    }
  }

  // Check reception buffer
  comm_receive();

//  // Print two newlines to make it easier to read
//  #ifdef SOFT_SERIAL_DEBUGGING
//    Serial.println("\n");
//  #endif
}

void Mav_Request_Data() {
  // Variable for the message, and the Serial buffer
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];


  // STREAMS that can be requested
  /*
     Definitions are in common.h: enum MAV_DATA_STREAM

     MAV_DATA_STREAM_ALL=0, // Enable all data streams
     MAV_DATA_STREAM_RAW_SENSORS=1, /* Enable IMU_RAW, GPS_RAW, GPS_STATUS packets.
     MAV_DATA_STREAM_EXTENDED_STATUS=2, /* Enable GPS_STATUS, CONTROL_STATUS, AUX_STATUS
     MAV_DATA_STREAM_RC_CHANNELS=3, /* Enable RC_CHANNELS_SCALED, RC_CHANNELS_RAW, SERVO_OUTPUT_RAW
     MAV_DATA_STREAM_RAW_CONTROLLER=4, /* Enable ATTITUDE_CONTROLLER_OUTPUT, POSITION_CONTROLLER_OUTPUT, NAV_CONTROLLER_OUTPUT.
     MAV_DATA_STREAM_POSITION=6, /* Enable LOCAL_POSITION, GLOBAL_POSITION/GLOBAL_POSITION_INT messages.
     MAV_DATA_STREAM_EXTRA1=10, /* Dependent on the autopilot
     MAV_DATA_STREAM_EXTRA2=11, /* Dependent on the autopilot
     MAV_DATA_STREAM_EXTRA3=12, /* Dependent on the autopilot
     MAV_DATA_STREAM_ENUM_END=13,

     Data in PixHawk available in:
      - Battery, amperage and voltage (SYS_STATUS) in MAV_DATA_STREAM_EXTENDED_STATUS
      - Gyro info (IMU_SCALED) in MAV_DATA_STREAM_EXTRA1
  */

  // To be setup according to the needed information to be requested from the Pixhawk
  // TODO - adjust these, because we don't necessarily need the extra one
  // TODO - adjust the rates, since we don't need them at 2 hz and 5 hz
  const int maxStreams = 2;
  const uint8_t MAVStreams[maxStreams] = {MAV_DATA_STREAM_EXTENDED_STATUS, MAV_DATA_STREAM_EXTRA1};
//  const uint16_t MAVRates[maxStreams] = {0x02, 0x05};
  const uint16_t MAVRates[maxStreams] = {0x01, 0x01}; // TODO - rate was adjusted


  // We only really need the first stream, as I don't know what the second stream is
  // The first stream contains the ground speed reading

  // Pack the message request together and prepare to send it, storing the bytes in "msg" variable
  // of type mavlink_message_t
  for (int i = 0; i < maxStreams; i++) {
    /*
       mavlink_msg_request_data_stream_pack(system_id, component_id,
          &msg,
          target_system, target_component,
          MAV_DATA_STREAM_POSITION, 10000000, 1);

       mavlink_msg_request_data_stream_pack(uint8_t system_id, uint8_t component_id,
          mavlink_message_t* msg,
          uint8_t target_system, uint8_t target_component, uint8_t req_stream_id,
          uint16_t req_message_rate, uint8_t start_stop)

    */
    mavlink_msg_request_data_stream_pack(2, 200, &msg, 1, 0, MAVStreams[i], MAVRates[i], 1);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    // Note: Writing the message is actually sending a request for data
    // TODO: Change this to always send the command via SoftSerial, and reserve Serial for USB comm
    // TODO: I think this request doesn't request the data once. Rather, it makes the Pixhawk continuously stream this output. IDK though
    #ifdef SOFT_SERIAL_DEBUGGING
        pxSerial.write(buf, len);
        Serial.println("Wrote request for data to the buffer (should happen ~once a minute)");
    #else
        Serial.write(buf, len);
    #endif
  }

  // Request: PARAM_REQUEST_LIST. Only for full log recording
  // TODO - this would be useful for debugging if the paramaeters are getting set correctly
  /*
     Primitive: mavlink_msg_param_request_list_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                   uint8_t target_system, uint8_t target_component)
  */
  /*
    // Configure
    uint8_t system_id=2;
    uint8_t component_id=200;
    // mavlink_message_t* msg;
    uint8_t target_system=1;
    uint8_t target_component=0;

    // Pack
    mavlink_msg_param_request_list_pack(system_id, component_id, &msg,
      target_system, target_component);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    // Send
    #ifdef SOFT_SERIAL_DEBUGGING
      pxSerial.write(buf,len);
    #else
      Serial.write(buf, len);
    #endif
  */
}


/**
 * Checks the reception buffer for MAVLink
 */
void comm_receive() {

  mavlink_message_t msg;
  mavlink_status_t status;

  // Use SoftSerial vs Serial as needed for debugging
  // TODO - improve this: REALLY bad idea (start of while loop within #ifdef, not closed within statement)
  #ifdef SOFT_SERIAL_DEBUGGING
    while (pxSerial.available() > 0) {
      uint8_t c = pxSerial.read();
  #else
    while (Serial.available() > 0) {
      uint8_t c = Serial.read();
  #endif

    // Try to get a new message
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {

      // Handle message
      switch (msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:  // #0: Heartbeat, TODO not currently doing anything with them
          // E.g. read GCS heartbeat and go into
          // comm lost mode if timer times out
          #ifdef SOFT_SERIAL_DEBUGGING
            //mySerial.println("PX HB");
          #endif
          break;

        case MAVLINK_MSG_ID_SYS_STATUS:  // #1: SYS_STATUS
          /* Message decoding: PRIMITIVE
                mavlink_msg_sys_status_decode(const mavlink_message_t* msg, mavlink_sys_status_t* sys_status)
          */
          //mavlink_message_t* msg;
          mavlink_sys_status_t sys_status;
          mavlink_msg_sys_status_decode(&msg, &sys_status);
          #ifdef SOFT_SERIAL_DEBUGGING
            Serial.print("PX SYS STATUS: ");
            Serial.print("[Bat (V): ");
            Serial.print(sys_status.voltage_battery);
            Serial.print("], [Bat (A): ");
            Serial.print(sys_status.current_battery);
            Serial.print("], [Comms loss (%): ");
            Serial.print(sys_status.drop_rate_comm);
            Serial.println("]");
          #endif
          break;

        case MAVLINK_MSG_ID_SYSTEM_TIME: // #2: SYSTEM_TIME
          // TODO NOW: I implemented this, so debug for any mistakes
          mavlink_system_time_t sys_time;
          mavlink_msg_system_time_decode(&msg, &sys_time);
          #ifdef SOFT_SERIAL_DEBUGGING
            mySerial.println("PX time: sys_time.time_boot_ms");
          break;

        case MAVLINK_MSG_ID_PARAM_VALUE:  // #22: PARAM_VALUE
          /* Message decoding: PRIMITIVE
               mavlink_msg_param_value_decode(const mavlink_message_t* msg, mavlink_param_value_t* param_value)
          */
          //mavlink_message_t* msg;
          mavlink_param_value_t param_value;
          mavlink_msg_param_value_decode(&msg, &param_value);
          #ifdef SOFT_SERIAL_DEBUGGING
            mySerial.println("PX PARAM_VALUE");
            mySerial.println(param_value.param_value);
            mySerial.println(param_value.param_count);
            mySerial.println(param_value.param_index);
            mySerial.println(param_value.param_id);
            mySerial.println(param_value.param_type);
            mySerial.println("------ Fin -------");
          #endif
          break;

        case MAVLINK_MSG_ID_RAW_IMU:  // #27: RAW_IMU
          /* Message decoding: PRIMITIVE
               static inline void mavlink_msg_raw_imu_decode(const mavlink_message_t* msg, mavlink_raw_imu_t* raw_imu)
          */
            mavlink_raw_imu_t raw_imu;
            mavlink_msg_raw_imu_decode(&msg, &raw_imu);
            #ifdef SOFT_SERIAL_DEBUGGING
              mySerial.print("PX Raw IMU, acc vector: [");
              mySerial.print(raw_imu.xacc); mySerial.print(", ");
              mySerial.print(raw_imu.yacc); mySerial.print(", ");
              mySerial.print(raw_imu.xacc); mySerial.println("]");
            #endif
          break;

        case MAVLINK_MSG_ID_ATTITUDE:  // #30
          /* Message decoding: PRIMITIVE
                mavlink_msg_attitude_decode(const mavlink_message_t* msg, mavlink_attitude_t* attitude)
          */
          mavlink_attitude_t attitude;
          mavlink_msg_attitude_decode(&msg, &attitude);
          #ifdef SOFT_SERIAL_DEBUGGING
            Serial.print("PX attitude, roll angle: ");
            Serial.println(attitude.roll);
          #endif
          break;


        default:
          #ifdef SOFT_SERIAL_DEBUGGING
            Serial.print("--- Other msg: ");
            Serial.print("[ID: ");
            Serial.print(msg.msgid);
            Serial.print("], [seq: ");
            Serial.print(msg.seq);
            Serial.println("]");
          #endif
          break;
      }
    }
  }
}
