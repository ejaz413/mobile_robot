#include <iostream>
#include <unistd.h>
#include <cstdlib>
#include "Definitions.h"

using namespace std;

typedef void* HANDLE;
typedef int BOOL;

// Function to check and print error
bool CheckError(const string& msg, unsigned int errorCode) {
    if (errorCode != 0) {
        cerr << "❌ " << msg << " ErrorCode: 0x" << hex << errorCode << endl;
        return false;
    }
    return true;
}

int main()
{
    // Bring up CAN interface (optional, only if Linux interface down)
    system("sudo ip link set can0 up type can bitrate 1000000");

    HANDLE keyHandle = 0;
    unsigned int errorCode = 0;

    string deviceName = "EPOS4";
    string protocolStackName = "CANopen";
    string interfaceName = "CAN_kvaser_usb 0";
    string portName = "CAN0";
    int baudrate = 1000000;

    unsigned short nodeId1 = 1;
    unsigned short nodeId2 = 2;

    cout << "Opening EPOS4 CANopen device..." << endl;

    keyHandle = VCS_OpenDevice(
        (char*)deviceName.c_str(),
        (char*)protocolStackName.c_str(),
        (char*)interfaceName.c_str(),
        (char*)portName.c_str(),
        &errorCode);

    if (keyHandle == 0 || errorCode != 0) {
        cerr << "❌ Failed to open device. ErrorCode: 0x" << hex << errorCode << endl;
        return -1;
    }

    cout << "Device opened successfully!" << endl;

    unsigned int lBaudrate = 0;
    unsigned int lTimeout = 0;
    if (VCS_GetProtocolStackSettings(keyHandle, &lBaudrate, &lTimeout, &errorCode) != 0) {
        VCS_SetProtocolStackSettings(keyHandle, baudrate, lTimeout, &errorCode);
        if (!CheckError("Setting protocol stack", errorCode)) return -1;
    }

    // Clear faults
    cout << "Clearing faults..." << endl;
    VCS_ClearFault(keyHandle, nodeId1, &errorCode);
    CheckError("Clearing fault Motor 1", errorCode);
    VCS_ClearFault(keyHandle, nodeId2, &errorCode);
    CheckError("Clearing fault Motor 2", errorCode);

    // Enable motors
    cout << "Enabling both motors..." << endl;
    VCS_SetEnableState(keyHandle, nodeId1, &errorCode);
    CheckError("Enabling Motor 1", errorCode);
    VCS_SetEnableState(keyHandle, nodeId2, &errorCode);
    CheckError("Enabling Motor 2", errorCode);

    // Verify motor states
    unsigned short state = 0;  // << change from unsigned int to unsigned short
    VCS_GetState(keyHandle, nodeId1, &state, &errorCode);
    CheckError("Getting Motor 1 state", errorCode);
    cout << "Motor 1 state: " << state << endl;

    VCS_GetState(keyHandle, nodeId2, &state, &errorCode);
    CheckError("Getting Motor 2 state", errorCode);
    cout << "Motor 2 state: " << state << endl;


    // Activate velocity mode
    cout << "Activating velocity mode..." << endl;
    VCS_ActivateProfileVelocityMode(keyHandle, nodeId1, &errorCode);
    CheckError("Activating velocity mode Motor 1", errorCode);
    VCS_ActivateProfileVelocityMode(keyHandle, nodeId2, &errorCode);
    CheckError("Activating velocity mode Motor 2", errorCode);

    long targetVelocity = 5000; // start smaller for testing

    cout << "Running motors for 2 seconds..." << endl;
    VCS_MoveWithVelocity(keyHandle, nodeId1, targetVelocity, &errorCode);
    CheckError("Moving Motor 1", errorCode);
    VCS_MoveWithVelocity(keyHandle, nodeId2, -targetVelocity, &errorCode);
    CheckError("Moving Motor 2", errorCode);

    sleep(2); // run for 2 seconds

    cout << "Stopping motors..." << endl;
    VCS_HaltVelocityMovement(keyHandle, nodeId1, &errorCode);
    CheckError("Halting Motor 1", errorCode);
    VCS_HaltVelocityMovement(keyHandle, nodeId2, &errorCode);
    CheckError("Halting Motor 2", errorCode);

    cout << "Disabling motors..." << endl;
    VCS_SetDisableState(keyHandle, nodeId1, &errorCode);
    CheckError("Disabling Motor 1", errorCode);
    VCS_SetDisableState(keyHandle, nodeId2, &errorCode);
    CheckError("Disabling Motor 2", errorCode);

    cout << "Closing device..." << endl;
    VCS_CloseDevice(keyHandle, &errorCode);
    CheckError("Closing device", errorCode);

    cout << "✅ Done. Both motors ran for 2 seconds." << endl;

    return 0;
}
