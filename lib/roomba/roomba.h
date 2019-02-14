#ifndef roomba_h
#define roomba_h

#include <SoftwareSerial.h>

#define BRC_PIN D4
#define BAUD_RATE 115200

#define LOGGING

class Roomba {
  public:
    SoftwareSerial *pSerial = NULL;

    // Roomba state
    typedef struct {
        // Sensor values
        int16_t distance;
        uint8_t chargingState;
        uint16_t voltage;
        int16_t current;
        // Supposedly unsigned according to the OI docs, but I've seen it
        // underflow to ~65000mAh, so I think signed will work better.
        int16_t charge;
        uint16_t capacity;

        // Derived state
        bool cleaning;
        bool docked;

        int timestamp;
        bool sent;
    } RoombaState;

    RoombaState roombaState = {};

    // Roomba sensor packet
    uint8_t roombaPacket[100];

    /// \enum PollState
    /// Values for _pollState
    typedef enum
    {
        PollStateIdle         = 0,
        PollStateWaitCount    = 1,
        PollStateWaitBytes    = 2,
        PollStateWaitChecksum = 3,
    } PollState;

    /// Variables for keeping track of polling of data streams
    uint8_t _pollState; /// Current state of polling, one of Roomba::PollState
    uint8_t _pollSize;  /// Expected size of the data stream in bytes
    uint8_t _pollCount; /// Num of bytes read so far
    uint8_t _pollChecksum; /// Running checksum counter of data bytes + count

  public:
    /// \enum Sensor
    /// Values for sensor packet IDs to pass to getSensors() and getSensorsList()
    typedef enum RoombaSensor
    {
        Sensors7to26             = 0,
        Sensors7to16             = 1,
        Sensors17to20            = 2,
        Sensors21to26            = 3,
        Sensors27to34            = 4,
        Sensors35to42            = 5,
        Sensors7to42             = 6,
        BumpsAndWheelDrops       = 7,
        Wall                     = 8,
        CliffLeft                = 9,
        CliffFrontLeft           = 10,
        CliffFrontRight          = 11,
        CliffRight               = 12,
        VirtualWall              = 13,
        Overcurrents             = 14,
    //	Unused1                  = 15,
    //	Unused2                  = 16,
        IRByte                   = 17,
        Buttons                  = 18,
        Distance                 = 19,
        Angle                    = 20,
        ChargingState            = 21,
        Voltage                  = 22,
        Current                  = 23,
        BatteryTemperature       = 24,
        BatteryCharge            = 25,
        BatteryCapacity          = 26,
        WallSignal               = 27,
        CliffLeftSignal           = 28,
        CliffFrontLeftSignal      = 29,
        CliffFrontRightSignal     = 30,
        CliffRightSignal          = 31,
        UserDigitalInputs        = 32,
        UserAnalogInput          = 33,
        ChargingSourcesAvailable = 34,
        OIMode                   = 35,
        SongNumber               = 36,
        SongPlaying              = 37,
        NumberOfStreamPackets    = 38,
        Velocity                 = 39,
        Radius                   = 40,
        RightVelocity            = 41,
        LeftVelocity             = 42
    } RoombaSensor;

    bool docked = false;

    // \enum ChargeState
    /// Values for sensor packet ID 21
    typedef enum
    {
        ChargeStateNotCharging            = 0,
        ChargeStateReconditioningCharging = 1,
        ChargeStateFullCharging           = 2,
        ChargeStateTrickleCharging        = 3,
        ChargeStateWaiting                = 4,
        ChargeStateFault                  = 5,
    } ChargeState;

  protected:
    bool parseRoombaStateFromStreamPacket(uint8_t *packet, int length, RoombaState *state);
    void verboseLogPacket(uint8_t *packet, uint8_t length);
    /// Polls the serial input for data belonging to a sensor data stream previously requested with stream().
    /// As sensor data is read it is appended to dest until at most len bytes are stored there. 
    /// When a complete sensor stream has been read with a correct checksum, returns true. 
    /// See the Open Interface manual for details on how the sensor data will be encoded in dest.
    /// Discards characters that are not part of a stream, such as the messages the Roomba 
    /// sends at startup and while charging.
    /// Create only. No equivalent on Roomba.
    /// \param[out] dest Destination where the read data is stored. Must have at least len bytes available.
    /// \param[out] packetLen Lenth of the read packet
    /// \param[in] len Max number of sensor data bytes to store to dest
    /// \return true when a complete stream has been read, and the checksum is correct. The sensor data
    /// (at most len bytes) will have been stored into dest, ready for the caller to decode.
    bool pollSensors(uint8_t* dest, uint8_t destSize, uint8_t *packetLen);

  public:
    Roomba();

    void set_baudrate(int speed);
    void wakeup(void);
    void wakeOnDock(void);
    void wakeOffDock(void);
    void power(void);
    void dock(void);
    void cover(void);
    void spot(void);
    void pause(void);
    void resume(void);
    void start(void);

    bool performMQTTCommand(const char *cmdchar);

    void readSensorPacket(void);
    void stream(const uint8_t* packetIDs, int len);

    inline bool isCleaning() { return roombaState.cleaning; };
    inline bool isDocked() { return roombaState.docked; };
    inline int getTimestamp() { return roombaState.timestamp; };
    inline bool isSent() { return roombaState.sent; };
};

#endif
