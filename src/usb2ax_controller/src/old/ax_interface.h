#ifndef AX_INTERFACE_H
#define AX_INTERFACE_H

#include "usb2ax_controller/SendToAX.h"

int main(int argc, char **argv);
bool getValue(int dxlID, int controlTableAddr, int& val);
bool setValue(int dxlID, int controlTableAddr, int val);
void printCommStatus(int CommStatus);
void printErrorCode(void);
bool sendToAX(usb2ax_controller::SendToAX::Request &req, usb2ax_controller::SendToAX::Response &res);

#endif // AX_INTERFACE_H

