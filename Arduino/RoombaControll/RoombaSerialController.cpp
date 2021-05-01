/**
 * @file  RoombaSerialController.cpp
 * @brief 
 * @sa 
 */

#include "RoombaSerialController.hpp"

RoombaSerialController::RoombaSerialController(){
    p_serialIF = nullptr;
}
RoombaSerialController::~RoombaSerialController(){

}
/* Setters */
void RoombaSerialController::setSerialInterface( SoftwareSerial* serialInterface )
{
    p_serialIF = serialInterface;
}

/* Functions */
void RoombaSerialController::Start( void )
{
    
}

void RoombaSerialController::DriveMotors( const int leftPwm, const int rightPwm )
{
    /* null check */
    if( isSerialInterfaceExist() == false ){
        return;
    }

    
}
void RoombaSerialController::TurnOnLed( void )
{

}
void RoombaSerialController::TurnOffLed( void )
{

}
/* SerialInterfaceChecker */
bool RoombaSerialController::isSerialInterfaceExist()
{
    if( p_serialIF == nullptr ){
        return false;
    }
    
    return true;
}