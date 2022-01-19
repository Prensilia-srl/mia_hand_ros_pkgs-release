/*********************************************************************************************************
Modifyed by Author: Prensilia srl
Desc:   Serial com port cpp hanlder.

version 1.0

**********************************************************************************************************/


#ifndef MIA_HAND_SERIAL_PORT_H
#define MIA_HAND_SERIAL_PORT_H

#include "libserial/SerialPort.h"
#include <mutex>

namespace mia_hand
{

/**
 * Struct containing all info that could regard a Mia hand motor.
 */
typedef struct FingerSerialInfo
{

  FingerSerialInfo();

  int16_t mot_pos;
  int8_t mot_spe;
  int16_t mot_cur;
  int16_t fin_sg_raw[2];
} FingerSerialInfo;

/**
 * Class to handle a serial port and its serial communication protocol.
 */
class SerialPort: public LibSerial::SerialPort
{
public:

  /**
   * Class destructor.
   */
  SerialPort(std::mutex* p_finger_data_mtx, std::mutex* p_connection_mtx,
             bool* p_is_connected);

 /**
  * Class destructor.
  */
  ~SerialPort();

  /**
   * Open a serial port.
   * @param port_num integer number of the serial port to open.
   */
  bool open(uint16_t port_num);

  /**
   * Close the serial port.
   */
  bool close();

  /**
   * Send a command to the Mia hand attached to the serial port.
   * This commands add to the input message the proper tail and send the composed message to the Mia hand.
   * @param command command to be sent.
   */
  void sendCommand(const std::string& command);

  /**
   * Parse message received from the Mia hand.
   * @param thumb Received info regarding the thumb flexion motor.
   * @param index Received info regarding the index flexion motor.
   * @param mrl Received info regarding the mrl flexion motor.
   * @param is_checking_on boolean to handle the check of the Mia hand connection.
   */
  void parseStream(FingerSerialInfo& thumb, FingerSerialInfo& index,
                   FingerSerialInfo& mrl, bool& is_checking_on);

private:

  std::string stream_msg_; //!< Message received from the Mia hand.

  std::mutex serial_write_mtx_;
  std::mutex* p_finger_data_mtx_;
  std::mutex* p_connection_mtx_;

  bool* p_is_connected_;
};
}  // namespace

#endif  // MIA_HAND_SERIAL_PORT_H
