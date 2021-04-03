/**
 * @file  RoombaSerialController.cpp
 * @brief Controll Roomba By SerialInterface
 * @sa SerialInterfaceDocument:https://www.irobot.lv/uploaded_files/File/iRobot_Roomba_500_Open_Interface_Spec.pdf
 * @date 2021/4/3 sibafb trys to implement basic functions.
 */
#ifndef ROOMBA_SERIAL_CONTROLLER_HPP_INCLUDED
#define ROOMBA_SERIAL_CONTROLLER_HPP_INCLUDED

#include <SoftwareSerial.h>;

class c{
    public:
        RoombaSerialController();
        ~RoombaSerialController();
        /* Setters */
        void setSerialInterface( SoftwareSerial& serialInterface );
        /* Functions */
        void DriveMotors( const int leftPwm , const int rightPwm );
        void TurnOnLed( void );
        void TurnOffLed( void );
    private:
        bool isSerialInterfaceExist();
        SoftwareSerial* p_serialIF;
}

#endif