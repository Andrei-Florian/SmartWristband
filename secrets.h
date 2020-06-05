// looping defines
#define RELEASE_TIME 300000 // 3 minutes
#define DEVELOP_TIME 10000  // 10 seconds

// Everything that has to be edited goes here
#define DEVICE_ID 1             // the ID of the device, needed when more devices are being connected to distinguish between them
#define SECRET_BROKER ""        // the iot hub name with .azure-devices.net at the end
#define SECRET_DEVICEID ""      // the name of the device
#define MODE DEVELOP_TIME       // change to RELEASE_TIME when deploying
