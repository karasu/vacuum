#include <Arduino.h>
#include <roomba.h>
#include <SoftwareSerial.h>

Roomba::Roomba()
{
    // RX, TX
    pSerial = new SoftwareSerial(D2, D3);
}

void Roomba::set_baudrate(int speed)
{
    pSerial->write(129);
    delay(50);
    pSerial->write(11);
    delay(50);
}

void Roomba::wakeup()
{
    pinMode(BRC_PIN, OUTPUT);
    digitalWrite(BRC_PIN, LOW);
    delay(200);
    pinMode(BRC_PIN, INPUT);
    delay(200);
    pSerial->write(128); // Start
}

void Roomba::wakeOnDock()
{
    this->wakeup();
    #ifdef ROOMBA_650_SLEEP_FIX
    // Some black magic from @AndiTheBest to keep the Roomba awake on the dock
    // See https://github.com/johnboiles/esp-roomba-mqtt/issues/3#issuecomment-402096638
    delay(10);
    pSerial->write(135); // Clean
    delay(150);
    pSerial->write(143); // Dock
    #endif
}

bool Roomba::performMQTTCommand(const char *cmdchar)
{
    wakeup();

    // Char* string comparisons dont always work
    String cmd(cmdchar);

    // MQTT protocol commands
    if (cmd == "turn_on") {
        cover();
        roombaState.cleaning = true;
    } else if (cmd == "turn_off") {
        power();
        roombaState.cleaning = false;
    } else if (cmd == "toggle" || cmd == "start_pause") {
        cover();
    } else if (cmd == "stop") {
        if (roombaState.cleaning) {
            cover();
        }
    } else if (cmd == "clean_spot") {
        roombaState.cleaning = true;
        spot();
    } else if (cmd == "locate") {
        // TODO
    } else if (cmd == "return_to_base") {
        roombaState.cleaning = true;
        dock();
    } else {
        return false;
    }
    return true;
}

void Roomba::wakeOffDock()
{
    pSerial->write(131); // Safe mode
    delay(300);
    pSerial->write(130); // Passive mode
}

void Roomba::power()
{
    pSerial->write(133);
}

void Roomba::dock()
{
    pSerial->write(143);
}

void Roomba::cover()
{
    pSerial->write(135);
}

void Roomba::spot()
{
    pSerial->write(134);
}

void Roomba::pause()
{
    pSerial->write(150);
    pSerial->write((uint8_t)0);
}

void Roomba::resume()
{
    pSerial->write(150);
    pSerial->write(1);
}

// Start OI
// Changes mode to passive
void Roomba::start()
{
    pSerial->begin(BAUD_RATE);
    pSerial->write(128);
    delay(100);
}

// States

bool Roomba::parseRoombaStateFromStreamPacket(uint8_t *packet, int length, RoombaState *state) {
  state->timestamp = millis();
  int i = 0;
  while (i < length) {
    switch(packet[i]) {
      case Roomba::Sensors7to26: // 0
        i += 27;
        break;
      case Roomba::Sensors7to16: // 1
        i += 11;
        break;
      case Roomba::VirtualWall: // 13
        i += 2;
        break;
      case Roomba::Distance: // 19
        state->distance = packet[i+1] * 256 + packet[i+2];
        i += 3;
        break;
      case Roomba::ChargingState: // 21
        state->chargingState = packet[i+1];
        i += 2;
        break;
      case Roomba::Voltage: // 22
        state->voltage = packet[i+1] * 256 + packet[i+2];
        i += 3;
        break;
      case Roomba::Current: // 23
        state->current = packet[i+1] * 256 + packet[i+2];
        i += 3;
        break;
      case Roomba::BatteryCharge: // 25
        state->charge = packet[i+1] * 256 + packet[i+2];
        i += 3;
        break;
      case Roomba::BatteryCapacity: //26
        state->capacity = packet[i+1] * 256 + packet[i+2];
        i += 3;
        break;
      case Roomba::BumpsAndWheelDrops: // 7
        i += 2;
        break;
      case 128: // Unknown
        i += 2;
        break;
      default:
        return false;
        break;
    }
  }
  return true;
}

// Simple state machine to read sensor data and discard everything else
bool Roomba::pollSensors(uint8_t* dest, uint8_t destSize, uint8_t *packetLen)
{
    while (pSerial->available())
    {
        uint8_t ch = pSerial->read();
        switch (_pollState)
        {
            case PollStateIdle:
                if (ch == 19)
                    _pollState = PollStateWaitCount;
                break;

            case PollStateWaitCount:
                _pollChecksum = _pollSize = ch;
                _pollCount = 0;
                _pollState = PollStateWaitBytes;
                break;

            case PollStateWaitBytes:
                _pollChecksum += ch;
                if (_pollCount < destSize)
                    dest[_pollCount] = ch;
                if (_pollCount++ >= _pollSize)
                    _pollState = PollStateWaitChecksum;
                break;

            case PollStateWaitChecksum:
                _pollChecksum += ch;
                _pollState = PollStateIdle;
                *packetLen = _pollSize;
                return (_pollChecksum == 0);
                break;
        }
    }
    return false;
}

void Roomba::readSensorPacket() {
  uint8_t packetLength;
  bool received = pollSensors(roombaPacket, sizeof(roombaPacket), &packetLength);
  if (received) {
    RoombaState rs = {};
    bool parsed = parseRoombaStateFromStreamPacket(roombaPacket, packetLength, &rs);
    if (parsed) {
      roombaState = rs;
      roombaState.cleaning = false;
      roombaState.docked = false;
      if (roombaState.current < -400) {
        roombaState.cleaning = true;
      } else if (roombaState.current > -50) {
        roombaState.docked = true;
      }
    }
  }
}

// Start a stream of sensor data with the specified packet IDs in it
void Roomba::stream(const uint8_t* packetIDs, int len)
{
    pSerial->write(148);
    pSerial->write((uint8_t)len);
    pSerial->write(packetIDs, len);
}
