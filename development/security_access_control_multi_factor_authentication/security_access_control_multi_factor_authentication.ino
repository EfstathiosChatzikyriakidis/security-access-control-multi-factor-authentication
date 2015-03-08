/*
 *  Security Access Control & Multi-Factor Authentication.
 *
 *  Copyright (C) 2011 TheHackerspace (info@the-hackerspace.org)
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * include external libraries headers.
 */

// library for creating finite state machines.
#include <FiniteStateMachine.h>

// library for creating software serial ports.
#include <SoftwareSerial.h>

// library for using keypad devices.
#include <Keypad.h>

/*
 * include internal libraries headers.
 */

// library for VRBot module communication protocol.
#include "vrbot_protocol.h"

// library for sound notes' frequencies.
#include "pitches.h"

/*
 * define preproccessor constants.
 */

// set true / false for debug support.
#define DEBUG_STATE false

// define controllers if debug state is on.
#if defined (DEBUG_STATE) && true == DEBUG_STATE
  // for hardware serial verbose printing.
  #define SERIAL_ENABLED
#endif

/*
 * define Arduino I/O PINS.
 */

// the PIN number for the relay access element of the system.
const byte ACCESS_RELAY_PIN = 13;

// the PIN number for the piezo status element of the system.
const byte STATUS_PIEZO_PIN = 10;

// the PIN number for the RFID reader serial TX line.
const byte RFID_RLINE_PIN = 9;

// the PIN number for the RFID reader serial RX line (dumb).
const byte RFID_TLINE_PIN = 10;

// the PIN number for the VRBot module serial TX line.
const byte VRBOT_RLINE_PIN = 12;

// the PIN number for the VRBot module serial RX line.
const byte VRBOT_TLINE_PIN = 11;

// the PIN numbers for the keypad rows.
byte KeypadRowPins[] = { 5, 4, 3, 2 };

// the PIN numbers for the keypad columns.
byte KeypadColPins[] = { 8, 7, 6 };

/*
 * define Arduino I/O serial port constants.
 */

#if defined (SERIAL_ENABLED)
// the hardware serial port baud rate.
const long USB_BAUD_RATE = 9600;
#endif

// the RFID software serial port baud rate.
const long RFID_BAUD_RATE = 9600;

// the VRBot software serial port baud rate.
const long VRBOT_BAUD_RATE = 9600;

/*
 * define FSM-related variables.
 */

// the states array for the FSM of the system.
State AuthSystemFSMStates[] = {
  State (RFIDFactorRoutine),
  State (KeypadFactorRoutine),
  State (VoiceFactorRoutine)
};

// the object to handle the FSM of the system.
FSM AuthSystemFSM = FSM (AuthSystemFSMStates[0]);

// calculate the number of total FSM states in the array.
const byte AUTH_SYSTEM_FSM_NUM_STATES =
  sizeof (AuthSystemFSMStates) / (sizeof (AuthSystemFSMStates[0]));

// the current FSM state number (zero-first, index-based).
byte AuthSystemFSMCurrentState = 0;

/*
 * define VRBot-related variables.
 */

// the period of a bit in the VRBot module communication.
const int VRBOT_BIT_PERIOD = 1000000 / VRBOT_BAUD_RATE;

// the delay time (ms) before VRBot module initialization.
const unsigned long VRBOT_DELAY_TIME_BEFORE_SETUP = 200;

// the timeout (secs) for the VRBot module initialization.
const byte VRBOT_TIMEOUT = 8;

// the language for the VRBot module initialization (0 = English).
const byte VRBOT_LANGUAGE = 0;

// the speaker dependent recognition group for the VRBot module.
const byte VRBOT_GROUP = 1;

/*
 * define RFID-related variables.
 */

// the object to control the RFID software serial port.
SoftwareSerial AuthSystemRFID = SoftwareSerial (RFID_RLINE_PIN, RFID_TLINE_PIN);

// the code size (digits) of the RFID tags.
const byte RFID_CODE_SIZE = 10;

// the user valid RFID code (+1 for string termination).
const char RFID_CODE_VALID[RFID_CODE_SIZE + 1] = "2500ABF859";

// the temporary RFID code (+1 for string termination).
char RFIDCodeTemp[RFID_CODE_SIZE + 1] = {'\0'};

/*
 * define Keypad-related variables.
 */

// calculate the number of keypad rows.
const byte KEYPAD_ROWS = sizeof (KeypadRowPins) / (sizeof (KeypadRowPins[0]));

// calculate the number of keypad columns.
const byte KEYPAD_COLS = sizeof (KeypadColPins) / (sizeof (KeypadColPins[0]));

// the keypad character set.
const char KEYPAD_KEYS[KEYPAD_ROWS][KEYPAD_COLS] = {
  {'1','2','3'},
  {'4','5','6'},
  {'7','8','9'},
  {'*','0','#'}
};

// the object to handle the keypad.
Keypad AuthSystemKeypad = Keypad(makeKeymap(KEYPAD_KEYS),
                                 KeypadRowPins,
                                 KeypadColPins,
                                 KEYPAD_ROWS,
                                 KEYPAD_COLS);

// the debounce delay time when pressing a key.
const word KEYPAD_DEBOUNCE_TIME = 250;

// the enter character where user presses after password insertion.
const char KEYPAD_ENTER_CHAR = '#';

// the necessary time to assume a key in hold state.
const word KEYPAD_HOLD_TIME = 500;

// the maximum length of the user password key.
const byte USER_PASSWORD_MAX_LENGTH = 32;

// the user valid password key.
const String USER_PASSWORD_VALID = "321*123";

// the user temporary password key.
String UserPasswordTemp = "";

/*
 * define Audio-related variables.
 */

// the happiness melody notes when access is granted.
const word NOTES_HAPPINESS[] = {
  NOTE_G4, NOTE_C4, NOTE_G3, NOTE_G3, NOTE_C4, NOTE_G3, NOTE_C4
};

// the number of the notes in the melody in the array.
const word NUM_NOTES = sizeof (NOTES_HAPPINESS) / sizeof (NOTES_HAPPINESS[0]);

// the note durations: 4 = quarter note, 8 = eighth note, etc.
const byte NOTES_DURATIONS[] = { 4, 8, 8, 2, 4, 2, 4 };

/*
 * define general variables.
 */

// the delay time when access is granted.
const unsigned long ACCESS_GRANTED_DELAY_TIME = 5000;

// the beep sound delay time when pressing a key.
const word BEEP_DELAY_TIME = 40;

/*
 * define Keypad-related functions.
 */

// keypad events handler function.
void
KeypadEventHandler(KeypadEvent Key) {
  // get the state of the keypad.
  switch (AuthSystemKeypad.getState()) {
    // if there is a key pressed.
    case PRESSED:
      // play a short beep sound.
      PlayBeep(BEEP_DELAY_TIME);
      break;

    // if there is a key released.
    case RELEASED:
      // do nothing - future additions.
      break;

    // if there is a key hold.
    case HOLD:
      // do nothing - future additions.
      break;
  }
}

/*
 * define Audio-related functions.
 */
 
// try to play a beep sound for some time period.
void
PlayBeep(const word Time) {
  // period of the wave signal (bigger means lower pitch).
  const word Period = 100;

  // calculate the duration of the beep sound.
  const word BeepDuration = (word) ((float) Time / Period * 1800);

  // play the beep sound (output the wave signal).
  for (word i = 0; i < BeepDuration; i++) {
    digitalWrite(STATUS_PIEZO_PIN, HIGH);
    delayMicroseconds(Period);

    digitalWrite(STATUS_PIEZO_PIN, LOW);
    delayMicroseconds(Period);
  }

  // delay some time.
  delay(Time);
}

// try to play a sad melody.
void
PlaySadness() {
  for (byte i = 0; i < 3; i++) {
    PlayBeep(BEEP_DELAY_TIME);
  }
}

// try to play a happy melody.
void
PlayHappiness() {
  // iterate over the notes of the melody.
  for (word ThisNote = 0; ThisNote < NUM_NOTES; ThisNote++) {
    // to calculate the note duration, take one second divided by the note type.

    // e.g. quarter note = 1000/4, eighth note = 1000/8, etc.
    word NoteDuration = 1000 / NOTES_DURATIONS[ThisNote];

    // play the tone.
    tone(STATUS_PIEZO_PIN, NOTES_HAPPINESS[ThisNote], NoteDuration);

    // to distinguish notes, set a minimum time between them.

    // the note's duration plus 30% seems to work well enough.
    word PauseBetweenNotes = NoteDuration * 1.30;

    // delay some time.
    delay(PauseBetweenNotes);
  }
}

/*
 * define RFID-related functions.
 */

// try to handle / parse an RFID tag.
bool
RFIDTagHandled () {
  byte Value = 0;       // temporary data received from RFID reader.
  byte Code[6];         // code + checksum data of RFID tag received.
  byte Checksum = 0;    // checksum data of RFID tag received.
  byte BytesRead = 0;   // number of received data from RFID reader.
  byte TempByte = 0;    // temporary value used for checksum calculation.
  bool Handled = false; // flag indicating if an RFID tag was handled.

  // if there are any data coming from the RFID reader.
  if (AuthSystemRFID.available () > 0) {
    // check for the STX header (0x02 ASCII value).
    if (0x02 == (Value = AuthSystemRFID.read ())) {
      // read the RFID 10-digit code & the 2 digit checksum.
      while (BytesRead < (RFID_CODE_SIZE + 2)) {
        // if there are any data coming from the RFID reader.
        if (AuthSystemRFID.available () > 0) {
          // get a byte from the RFID reader.
          Value = AuthSystemRFID.read ();

          // check for ETX, STX, CR, LF.
          if ((0x0D == Value) ||
              (0x0A == Value) ||
              (0x03 == Value) ||
              (0x02 == Value)) {
            // stop reading - there is an error.
            break;
          }

          // store the RFID code digits to an array.
          if (BytesRead < RFID_CODE_SIZE)
            RFIDCodeTemp[BytesRead] = Value;

          // convert hex tag ID.
          if ((Value >= '0') && (Value <= '9'))
            Value = Value - '0';
          else if ((Value >= 'A') && (Value <= 'F'))
            Value = 10 + Value - 'A';

          // every two hex-digits, add byte to code.
          if (BytesRead & 1 == 1) {
            // make some space for this hex-digit by shifting
            // the previous hex-digit with 4 bits to the left.
            Code[BytesRead >> 1] = (Value | (TempByte << 4));

            if (BytesRead >> 1 != 5)
              // if we're at the checksum byte, calculate the checksum (XOR).
              Checksum ^= Code[BytesRead >> 1];
          }
          else
            TempByte = Value;

          // ready to read next byte.
          BytesRead++;
        }
      }

      // handle the RFID 10-digit code & the 2 digit checksum data.
      if (BytesRead == (RFID_CODE_SIZE + 2)) {
        // check if the RFID code is correct.
        if (Code[5] == Checksum)
          // set that the tag was handled.
          Handled = true;
      }
    }
  }

  return Handled;
}

/*
 * define utilities functions.
 */

// try to set the PIN to 'high' value with delay time.
void
SetPinHighWithDelay(const byte Pin, const unsigned long Time) {
  digitalWrite(Pin, HIGH);
  delay(Time);
}

// try to set the PIN to 'low' value with delay time.
void
SetPinLowWithDelay(const byte Pin, const unsigned long Time) {
  digitalWrite(Pin, LOW);
  delay(Time);
}

// try to perform an infinite warning error state.
void
CriticalError () {
  while (true) {
    PlaySadness ();
  }
}

// stream flush function.
void
StreamFlush (Stream & stream)
{
  // perform old behaviour of flush (before Arduino 1.0).
  while (stream.available ()) {
    stream.read ();
  }
}

/*
 * define VRBot-related functions.
 */

// try to read data from the VRBot module.
byte
VRBotRead () {
  // the data that will be read.
  byte Value = 0;

  // digital read delay is about 100 cycles.
  int BitDelay = VRBOT_BIT_PERIOD - clockCyclesToMicroseconds(100);
  
  // one byte of serial data (LSB first).

  // bits: start, 0, 1, 2, 3, 4, 5, 6, 7, stop.

  while (digitalRead(VRBOT_RLINE_PIN));

  // confirm that this is a real start bit, not line noise.
  if (digitalRead(VRBOT_RLINE_PIN) == LOW) {
    // frame start indicated by a falling edge and low start bit.

    // jump to the middle of the low start bit.
    delayMicroseconds(BitDelay / 2 - clockCyclesToMicroseconds(50));

    // offset of the bit in the byte: from 0 (LSB) to 7 (MSB).
    for (int Offset = 0; Offset < 8; Offset++) {
      // jump to middle of next bit.
      delayMicroseconds(BitDelay);

      // read the bit.
      Value |= digitalRead(VRBOT_RLINE_PIN) << Offset;
    }

    delayMicroseconds(VRBOT_BIT_PERIOD);

    // return the data.    
    return Value;
  }

  return -1;
}

// try to write data to the VRBot module.
void
VRBotWrite (byte Value) {
  // digital write delay is about 50 cycles.
  int BitDelay = VRBOT_BIT_PERIOD - clockCyclesToMicroseconds(50);

  digitalWrite(VRBOT_TLINE_PIN, LOW);
  delayMicroseconds(BitDelay);

  for (byte Mask = 0x01; Mask; Mask <<= 1) {
    // choose bit.
    if (Value & Mask)
      digitalWrite(VRBOT_TLINE_PIN, HIGH);
    else
      digitalWrite(VRBOT_TLINE_PIN, LOW);

    delayMicroseconds(BitDelay);
  }

  digitalWrite(VRBOT_TLINE_PIN, HIGH);
  delayMicroseconds(BitDelay);
}

// try to recognize a speaker dependent group from VRBot module.
void
VRBotRecognizeSD(byte Group) {
  VRBotWrite(CMD_RECOG_SD);
  delay(5);
  VRBotWrite(ARG_ZERO + Group);
}

// try to check the result from a possible VRBot recognition.
char
VRBotCheckResult() {
  byte rx = VRBotRead();

  if (rx == STS_SIMILAR || rx == STS_RESULT) {
    delay(5);

    VRBotWrite(ARG_ACK);

    // return command recognized.
    return (VRBotRead() - ARG_ZERO);
  }

  // timeout return code.
  if (rx == STS_TIMEOUT)
    return -1;
    
  // error return code.
  return -2;
}

// try to recognize the SD message.
bool
SDRecognition() {
  // speaker recognized command.
  int Cmd;

  // start SD message recognition.
  VRBotRecognizeSD (VRBOT_GROUP);

  // check recognition result.
  Cmd = VRBotCheckResult ();

  // check for timeout result.
  if (Cmd == -1) return false;
  
  // check for error result.
  if (Cmd == -2) return false;

  // recognition succeeded.
  return true;
}

// try to setup the VRBot module.
void
VRBotSetup() {
  digitalWrite(VRBOT_TLINE_PIN, HIGH);
  delayMicroseconds(VRBOT_BIT_PERIOD); 
}

// try to detect the VRBot module.
byte
VRBotDetect() {
  for (byte i = 0; i < 5; ++i) {
    delay(100);    
    VRBotWrite(CMD_BREAK);
    if (VRBotRead() == STS_SUCCESS)
      return 255;
  }

  return 0;
}

// try to set the language for the VRBot module.
byte
VRBotSetLanguage(byte Lang) {
  VRBotWrite(CMD_LANGUAGE);
  delay(5);
  VRBotWrite(ARG_ZERO + Lang);

  if (VRBotRead() == STS_SUCCESS)
    return 255;

  return 0;
}

// try to set the timeout of the VRBot module.
void
VRBotSetTimeout(byte Seconds) {
  VRBotWrite(CMD_TIMEOUT);
  delay(5);

  VRBotWrite(ARG_ZERO + Seconds);
  delay(5);
}

/*
 * define FSM-related functions.
 */

// try to check if the authentication is totally completed.
bool
AuthenticationCompleted () {
  if (AuthSystemFSMCurrentState == AUTH_SYSTEM_FSM_NUM_STATES)
    return true;

  return false;
}

// try to perform the appropriate operations when access is granted.
void
GiveAccessOperations () {
  // open the relay with some delay time.
  SetPinHighWithDelay (ACCESS_RELAY_PIN, ACCESS_GRANTED_DELAY_TIME);

  // close the relay with no delay time.
  SetPinLowWithDelay (ACCESS_RELAY_PIN, 0);
}

// try to set the next state of the FSM.
void
AuthSystemFSMSetNextState () {
  // since the FSM is linear set the next state sequentially.
  AuthSystemFSMCurrentState++;

  // check if the authentication is totally completed.
  if (AuthenticationCompleted ())
    GiveAccessOperations ();

  // if the FSM is completed then the first state is selected again (cyclic behaviour).
  AuthSystemFSMCurrentState %= AUTH_SYSTEM_FSM_NUM_STATES;
}

// Keypad Authentication Factor Routine.
void
KeypadFactorRoutine () {
  // try to get a key from the keypad.
  char Key = AuthSystemKeypad.getKey();

  // if there was a normal key.
  if (Key != NO_KEY) {
    // if the user didn't typed enter.
    if (Key != KEYPAD_ENTER_CHAR) {
      // if the user does stress testing with the password length.
      if (UserPasswordTemp.length() >= USER_PASSWORD_MAX_LENGTH) {
        // play a sadness sound since the try was wrong.
        PlaySadness ();

        // empty the user temporary password for the next try.
        UserPasswordTemp = "";
      }
      else
        // append the key to the buffer.
        UserPasswordTemp += Key;
    }
    else {
      // if the password the user typed is the valid one.
      if (UserPasswordTemp == USER_PASSWORD_VALID) {
        // play a happiness sound since the try was correct.
        PlayHappiness ();

        // set the next state.
        AuthSystemFSMSetNextState ();

        // print an information message.
        #if defined (SERIAL_ENABLED)
        Serial.println("Keypad OK!");
        #endif
      }
      else
        // play a sadness sound since the try was wrong.
        PlaySadness ();

      // empty the user temporary password for the next try.
      UserPasswordTemp = "";
    }
  }

  // ensure that no RFID data waiting in buffer.
  StreamFlush(AuthSystemRFID);
}

// RFID Authentication Factor Routine.
void
RFIDFactorRoutine () {
  // try to handle an RFID tag.
  if (RFIDTagHandled ()) {
    // if the RFID code is the valid one.
    if (!strcmp (RFIDCodeTemp, RFID_CODE_VALID)) {
      // play a happiness sound since the try was correct.
      PlayHappiness ();

      // set the next state.
      AuthSystemFSMSetNextState ();

      // print an information message.
      #if defined (SERIAL_ENABLED)
      Serial.println("RFID OK!");
      #endif
    }
    else
      // play a sadness sound since the try was wrong.
      PlaySadness ();
  }
}

// Voice Authentication Factor Routine.
void
VoiceFactorRoutine () {
  // try to recognize the SD message.
  if (SDRecognition()) {
    // play a happiness sound since the try was correct.
    PlayHappiness ();

    // set the next state.
    AuthSystemFSMSetNextState ();

    // print an information message.
    #if defined (SERIAL_ENABLED)
    Serial.println("Voice OK!");
    #endif
  }
  else
    // play a sadness sound since the try was wrong.
    PlaySadness ();

  // ensure that no RFID data waiting in buffer.
  StreamFlush(AuthSystemRFID);
}

/*
 * define setup & loop functions.
 */

// startup point entry (runs once).
void
setup () {
  #if defined (SERIAL_ENABLED)
  // set hardware serial port data rate.
  Serial.begin (USB_BAUD_RATE);
  #endif

  // set RFID serial port data rate.
  AuthSystemRFID.begin (RFID_BAUD_RATE);

  // set VRBot UART PINS I/O mode.
  pinMode(VRBOT_RLINE_PIN, INPUT);
  pinMode(VRBOT_TLINE_PIN, OUTPUT);

  // set the status piezo PIN as output.
  pinMode(STATUS_PIEZO_PIN, OUTPUT);

  // set the access relay PIN as output.
  pinMode(ACCESS_RELAY_PIN, OUTPUT);

  // set some keypad related options.
  AuthSystemKeypad.addEventListener(KeypadEventHandler);
  AuthSystemKeypad.setDebounceTime(KEYPAD_DEBOUNCE_TIME);
  AuthSystemKeypad.setHoldTime(KEYPAD_HOLD_TIME);

  // delay some time before the VRBot setup.
  delay(VRBOT_DELAY_TIME_BEFORE_SETUP);
  
  // try to setup the VRBot module.
  VRBotSetup();

  // try to detect the VRBot module.
  if (!VRBotDetect())
     // perform an infinite warning error state.
     CriticalError ();
  else {
    // set the VRBot timeout.
    VRBotSetTimeout(VRBOT_TIMEOUT);

    // set the VRBot language.
    VRBotSetLanguage(VRBOT_LANGUAGE);  
  }
}

// loop the main sketch.
void
loop () {
  // schedule a state according to the FSM.
  AuthSystemFSM.transitionTo (AuthSystemFSMStates[AuthSystemFSMCurrentState]);

  // update the FSM to call the appropriate state routine.
  AuthSystemFSM.update ();
}
