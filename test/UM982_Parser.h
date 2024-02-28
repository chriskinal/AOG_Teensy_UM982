/*Copyright 2023 Chris Kinal

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this 
ist of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS “AS IS”
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/*
 * Unicore specific message parser library for Arduino
 *
 * Based upon the Simple and compact NMEA parser designed for Arduino by Glinnes Hulden.
 *
*/

// Error codes
// 0 - UM982::NO_ERROR: no error;
// 1 - UM982::UNEXPECTED_CHAR: a char which is not expected in the sentence has been encountered;
// 2 - UM982::BUFFER_FULL: the sentence is too long to fit in the buffer;
// 3 - UM982::TYPE_TOO_LONG: the sentence type has more than 5 characters;
// 4 - UM982::CRC_ERROR: the CRC is wrong;
// 5 - UM982::INTERNAL_ERROR: the internal state of the parser is wrong, should not happen by the way.

#ifndef __UM982Parser_h__
#define __UM982Parser_h__

#include <Arduino.h>

namespace UM982 {
  /*
   * Error codes
   */
  typedef enum {
      NO_ERROR,
      UNEXPECTED_CHAR,
      BUFFER_FULL,
      TYPE_TOO_LONG,
      CRC_ERROR,
      INTERNAL_ERROR
  } ErrorCode;
}

/*
 * The library consists of a single template: UM982Parser.
 */
template <size_t S> class UM982Parser {

private:
  typedef void (*UM982ErrorHandler)(void);
  typedef void (*UM982Handler)(void);
  typedef struct { char mToken[12]; UM982Handler mHandler; } UM982HandlerEntry;
  typedef enum { INIT, SENT, ARG, CRC1, CRC2, CRC3, CRC4, CRC5, CRC6, CRC7, CRC8, CRLFCR, CRLFLF } State;
public:
  /*
   * maximum sentence size is 254 including the starting '#' and the <cr><lf>
   * at the end.
   */
  
  static const uint16_t kSentenceMaxSize = 768;

private:
  /*
   * buffer to store the UM982 sentence excluding starting '#', the ','
   * separator, the '*', the ';', the CRC and the <cr><lf>.
   */
  char mBuffer[kSentenceMaxSize];
  String message;

  /*
   * Buffer to store the UM982 message portion of the sentence for CRC calculation.
  */
  char mCRCBuffer[kSentenceMaxSize];

  /*
   Buffer to store the UM982 CRC32 hash from the sentence.
  */
  char mCRC32[9];

  /*
   * Current index to store a char of the sentence in mBuffer
   */
  uint16_t mIndex;

   /*
   * Buffer for sentence to calculate CRC32
   */
  uint16_t mCRCBufIndex;

  /*
   * Index for sentence to calculate CRC32
   */
  uint8_t mCRC32Index;
  
  /*
   * Array of int's to store argument start and end positions
   */
  uint16_t mArgArray[100];

  /*
   * Current argument index to store the mBuffer index of an argument
   */
  uint16_t mArgIndex;

  /*
   * A handler to notify a malformed sentence
   */
  UM982ErrorHandler mErrorHandler;

  /*
   * A handler for well formed but unrecognized sentences
   */
  UM982Handler mDefaultHandler;

  /*
   * An array of UM982 handler : pointers to functions to call when a sentence
   * is recognized
   */
  UM982HandlerEntry mHandlers[S];

  /*
   * The current number of mHandlers
   */
  uint8_t mHandlerCount;

  /*
   * Parsing automaton variable
   */
  State mState;

  /*
   * mError
   */
  UM982::ErrorCode mError;

  /*
   * True if CRC is handled, false otherwise. Defaults to true
   */
  bool mHandleCRC;

   /*
   * UM982ParserStringify is used internally to temporarely replace a char
   * in the buffer by a '\0' so that libc string functions may be used.
   * Instantiating a UM982ParserStringify object in a pair of {} defines
   * a section in which the 'stringification' is done : the constructor
   * does that according to the arguments and the destructor restore the buffer.
   */
  class UM982ParserStringify {
    uint16_t       mPos;
    char          mTmp;
    UM982Parser<S> *mParent;
  public:
    UM982ParserStringify(UM982Parser<S> *inParent, uint16_t inPos) :
      mPos(inPos),
      mParent(inParent)
    {
      mTmp = mParent->mBuffer[mPos];
      mParent->mBuffer[mPos] = '\0';
    }
    ~UM982ParserStringify()
    {
      mParent->mBuffer[mPos] = mTmp;
    }
  };

  /*
   * Call the error handler if defined
   */
  void callErrorHandler(void)
  {
    if (mErrorHandler != NULL) {
      mErrorHandler();
    }
  }

  /*
   * Called when the parser encounter a char that should not be there
   */
  void unexpectedChar()
  {
    mError = UM982::UNEXPECTED_CHAR;
    callErrorHandler();
    reset();
  }

  /*
   * Called when the buffer is full because of a malformed sentence
   */
  void bufferFull()
  {
    mError = UM982::BUFFER_FULL;
    callErrorHandler();
    reset();
  }

  /*
   * Called when the type of the sentence is longer than 13 characters
   */
  void typeTooLong()
  {
    mError = UM982::TYPE_TOO_LONG;
    callErrorHandler();
    reset();
  }
  /*
   * Called when the CRC is wrong
   */
  void crcError()
  {
    mError = UM982::CRC_ERROR;
    callErrorHandler();
    reset();
  }

  /*
   * Called when the state of the parser is not ok
   */
  void internalError()
  {
    mError = UM982::INTERNAL_ERROR;
    callErrorHandler();
    reset();
  }

  /*
   * retuns true if there is at least one byte left in the buffer
   */
  bool spaceAvail()
  {
    return (mIndex < kSentenceMaxSize - 1);
  }

  /*
  * Check if character is in the Unicomm character, number or punctuation set
  */
  static int isUnichars(const char inChar)
  {
    if (isalnum(inChar)) return 1;
    else if (inChar == '.') return 1;
    else if (inChar == '-') return 1;
    else return 0;
  }

  /*
   * convert a one hex digit char into an int. Used for the CRC
   */
  static int8_t hexToNum(const char inChar)
  {
    if (isdigit(inChar)) return inChar - '0';
    else if (isupper(inChar) && inChar <= 'F') return inChar - 'A' + 10;
    else if (islower(inChar) && inChar <= 'f') return inChar - 'a' + 10;
    else return -1;
  }

  static bool strnwcmp(const char *s1, const char *s2, uint8_t len)
  {
    while (len-- > 0) {
      if (*s1 != *s2 && *s1 != '-' && *s2 != '-') return false;
      s1++;
      s2++;
    }
    return true;
  }

  /*
   * return the slot number for a handler. -1 if not found
   */
  int8_t getHandler(const char *inToken)
  {
    /* Look for the token */
   for (uint8_t i = 0; i < mHandlerCount; i++) {
      if (strnwcmp(mHandlers[i].mToken, inToken, strlen(inToken))) {
        return i;
      }
    }
    return -1;
  }

  /*
   * When all the sentence has been parsed, process it by calling the handler
   */
  void processSentence()
  {
    /* Look for the token */
    uint16_t endPos = startArgPos(0);
    int16_t slot;

    {
      UM982ParserStringify stfy(this, endPos);
      slot = getHandler(mBuffer);
    }
    if (slot != -1) {
      mHandlers[slot].mHandler();
    }
    else {
      if (mDefaultHandler != NULL) {
        mDefaultHandler();
      }
    }
  }

  /*
   * Return true if inArgNum corresponds to an actual argument
   */
  bool validArgNum(uint16_t inArgNum)
  {
    return mArgArray[inArgNum];
  }

  /*
   * Return the start index of the inArgNum argument
   */
  uint16_t startArgPos(uint16_t inArgNum)
  {
    return mArgArray[inArgNum];
  }

  /*
   * Return the end index of the inArgNum argument
   */
  uint16_t endArgPos(uint16_t inArgNum)
  {
    return mArgArray[inArgNum + 1];
  }

public:
  /*
   * Constructor initialize the parser.
   */
  UM982Parser() :
    mErrorHandler(NULL),
    mDefaultHandler(NULL),
    mHandlerCount(0),
    mError(UM982::NO_ERROR),
    mHandleCRC(true)
  {
    reset();
  }

  /*
 * Reset the parser
 */
  void reset() {
      mState = INIT;
      message = "";
      mIndex = 0;
      mCRCBufIndex = 0;
      memset(mCRCBuffer, 0, kSentenceMaxSize);
      memset(mBuffer, 0, kSentenceMaxSize);
      memset(mArgArray, 0, 100);
      mCRC32Index = 0;
      mArgIndex = 0;
      mError = UM982::NO_ERROR;
  }

  /*
   * Add a sentence handler
   */
  void addHandler(const char *inToken, UM982Handler inHandler)
  {
    if (mHandlerCount < S) {
      if (getHandler(inToken) == -1) {
        strncpy(mHandlers[mHandlerCount].mToken, inToken, strlen(inToken));
        mHandlers[mHandlerCount].mToken[strlen(inToken) + 1] = '\0';
        mHandlers[mHandlerCount].mHandler = inHandler;
        mHandlerCount++;
      }
    }
  }

#ifdef __AVR__
  /*
   * Add a sentence handler. Version with a token stored in flash.
   */
   void addHandler(const __FlashStringHelper *ifsh, UM982Handler inHandler)
   {
     char buf[6];
     PGM_P p = reinterpret_cast<PGM_P>(ifsh);
     for (uint8_t i = 0; i < 6; i++) {
       char c = pgm_read_byte(p++);
       buf[i] = c;
       if (c == '\0') break;
     }
     addHandler(buf, inHandler);
   }
#endif

  /*
   * Set the error handler which is called when a sentence is malformed
   */
  void setErrorHandler(UM982ErrorHandler inHandler)
  {
    mErrorHandler = inHandler;
  }

  /*
   * Set the default handler which is called when a sentence is well formed
   * but has no handler associated to
   */
  void setDefaultHandler(UM982Handler inHandler)
  {
    mDefaultHandler = inHandler;
  }

  /*
   * Give a character to the parser
   */
  void operator<<(char inChar)
  {
    int8_t tmp;
    message = message + inChar;
    switch (mState) {

      /* Waiting for the starting $ character */
      case INIT:
        mError = UM982::NO_ERROR;
        if ( inChar == '#' ) {
          mState = SENT;
        }
        else
          {
            unexpectedChar();
            //Serial.println("Unexchar1");
          }
        break;

      case SENT:
        mCRCBuffer[mCRCBufIndex++] = inChar;
        if ( (isalnum(inChar)) ) {
          if (spaceAvail()) {
            if (mIndex < 14) {
              mBuffer[mIndex++] = inChar;
              }
            else {
              typeTooLong();
            }
          }
          else bufferFull();
        }
        else {
          switch(inChar) {
            case ',' :
              mArgArray[mArgIndex] = mIndex;
              mArgIndex = mArgIndex + 1;
              mState = ARG;
              break;
            case '*' :
              mArgArray[mArgIndex] = mIndex;
              mState = CRC1;
              break;
            default :
              unexpectedChar();
              //Serial.println("Unexchar2");
              break;
          }
        }
        break;

      case ARG:
        if (spaceAvail()) {
          switch(inChar) {
            case ',' :
              mCRCBuffer[mCRCBufIndex++] = inChar;
              mArgArray[mArgIndex++] = mIndex;
              break;
            case ';' :
              mCRCBuffer[mCRCBufIndex++] = inChar;
              mArgArray[mArgIndex++] = mIndex;
            break;
            case '*' :
              mArgArray[mArgIndex++] = mIndex;
              mState = CRC1;
              break;
            default :
              mCRCBuffer[mCRCBufIndex++] = inChar;
              mBuffer[mIndex++] = inChar;
              break;
          }
        }
        else bufferFull();
        break;

      case CRC1:
        tmp = hexToNum(inChar);
        if (tmp != -1) {
          mCRC32[mCRC32Index++] = inChar;
          mState = CRC2;
        }
        else
          {
            unexpectedChar();
            //Serial.println("Unexchar3");
          }
        break;

      case CRC2:
        tmp = hexToNum(inChar);
        if (tmp != -1) {
          mCRC32[mCRC32Index++] = inChar;
          mState = CRC3;
        }
        else 
          {
            unexpectedChar();
            //Serial.println("Unexchar4");
          }
        break;

      case CRC3:
        tmp = hexToNum(inChar);
        if (tmp != -1) {
          mCRC32[mCRC32Index++] = inChar;
          mState = CRC4;
        }
        else 
          {
            unexpectedChar();
            //Serial.println("Unexchar5");
          }
        break;

      case CRC4:
        tmp = hexToNum(inChar);
        if (tmp != -1) {
          mCRC32[mCRC32Index++] = inChar;
          mState = CRC5;
        }
        else 
          {
            unexpectedChar();
            //Serial.println("Unexchar6");
          }
        break;

      case CRC5:
        tmp = hexToNum(inChar);
        if (tmp != -1) {
          mCRC32[mCRC32Index++] = inChar;
          mState = CRC6;
        }
        else 
          {
            unexpectedChar();
            //Serial.println("Unexchar7");
          }
        break;

      case CRC6:
        tmp = hexToNum(inChar);
        if (tmp != -1) {
          mCRC32[mCRC32Index++] = inChar;
          mState = CRC7;
        }
        else 
          {
            unexpectedChar();
            //Serial.println("Unexchar8");
          }
        break;

      case CRC7:
        tmp = hexToNum(inChar);
        if (tmp != -1) {
          mCRC32[mCRC32Index++] = inChar;
          mState = CRC8;
        }
        else 
          {
            unexpectedChar();
            //Serial.println("Unexchar9");
          }
        break;

      case CRC8:
        tmp = hexToNum(inChar);
        if (tmp != -1) {
          mCRC32[mCRC32Index++] = inChar;
          mState = CRLFCR;
        }
        else 
          {
            unexpectedChar();
            //Serial.println("Unexchar10");
          }
        break;

      case CRLFCR:
        if (inChar == '\r') {
          mState = CRLFLF;
        }
        else
          {
            unexpectedChar();
            //Serial.println("Unexchar11");
          }
        break;

      case CRLFLF:
        if (inChar == '\n') {
          unsigned long uniCRC = strtoul(mCRC32, 0, 16);
          unsigned long msgCRC = CalculateCRC32(mCRCBuffer, strlen(mCRCBuffer));
          if (mHandleCRC && (uniCRC != msgCRC)) {
            crcError();
          }
          else {
            processSentence();
          }
          reset();
        }
        else 
        {
          unexpectedChar();
          //Serial.println("Unexchar5");
        }
        break;

      default:
        internalError();
        break;
    }
  }

  /*
   * Returns the number of arguments discovered in a well formed sentence.
   */
  uint8_t argCount()
  {
    for (int i = 1; i < 100; i++){
      if (mArgArray[i] == 0){return i - 1;}
    }
  }

  /*
   * Returns one of the arguments. Different versions according to data type.
   */
  bool getArg(uint16_t num, char &arg)
  {
    if (validArgNum(num)) {
      uint16_t startPos = startArgPos(num);
      uint16_t endPos = endArgPos(num);
      arg = mBuffer[startPos];
      return (endPos - startPos) == 1;
    }
    else return false;
  }

  bool getArg(uint16_t num, char *arg)
  {
    if (validArgNum(num)) {
      uint16_t startPos = startArgPos(num);
      uint16_t endPos = endArgPos(num);
      {
        UM982ParserStringify stfy(this, endPos);
        strcpy(arg, &mBuffer[startPos]);
      }
      return true;
    }
    else return false;
  }

#ifdef ARDUINO
  bool getArg(uint16_t num, String &arg)
  {
    if (validArgNum(num)) {
      uint16_t startPos = startArgPos(num);
      uint16_t endPos = endArgPos(num);
      {
        UM982ParserStringify stfy(this, endPos);
        arg = &mBuffer[startPos];
      }
      return true;
    }
    else return false;
  }
#endif

  bool getArg(uint16_t num, int &arg)
  {
    if (validArgNum(num)) {
      uint16_t startPos = startArgPos(num);
      uint16_t endPos = endArgPos(num);
      {
        UM982ParserStringify stfy(this, endPos);
        arg = atoi(&mBuffer[startPos]);
      }
      return true;
    }
    else return false;
  }

  bool getArg(uint16_t num, float &arg)
  {
    if (validArgNum(num)) {
      uint16_t startPos = startArgPos(num);
      uint16_t endPos = endArgPos(num);
      {
        UM982ParserStringify stfy(this, endPos);
        arg = atof(&mBuffer[startPos]);
      }
      return true;
    }
    else return false;
  }

 bool getArg(uint16_t num, double &arg)
  {
    if (validArgNum(num)) {
      uint16_t startPos = startArgPos(num);
      uint16_t endPos = endArgPos(num);
      {
        UM982ParserStringify stfy(this, endPos);
        arg = strtod(&mBuffer[startPos], NULL);
      }
      return true;
    }
    else return false;
  }

  /*
   * Returns the type of sentence.
   */
  bool getType(char *arg)
  {
    if (mIndex > 0) {
      uint16_t endPos = startArgPos(0);
      {
        UM982ParserStringify stfy(this, endPos);
        strncpy(arg, mBuffer, 12);
        arg[12] = '\0';
      }
      return true;
    }
    else return false;
  }

#ifdef ARDUINO
  bool getType(String &arg)
  {
    if (mIndex > 0) {
      uint16_t endPos = startArgPos(0);
      {
        UM982ParserStringify stfy(this, endPos);
        arg = mBuffer;
      }
      return true;
    }
    else return false;
  }
#endif

  bool getType(uint16_t inIndex, char &outTypeChar)
  {
    if (mIndex > 0) {
      uint16_t endPos = startArgPos(0);
      if (inIndex < endPos) {
        outTypeChar = mBuffer[inIndex];
        return true;
      }
      else return false;
    }
    else return false;
  }

  UM982::ErrorCode error() {
    return mError;
  }

  void setHandleCRC(bool inHandleCRC)
  {
    mHandleCRC = inHandleCRC;
  }
#ifdef __amd64__
  void printBuffer()
  {
    {
      UM982ParserStringify stfy(this, startArgPos(0));
      printf("%s\n", mBuffer);
    }
    for (uint8_t i = 0; i < argCount(); i++) {
      uint8_t startPos = startArgPos(i);
      uint8_t endPos = endArgPos(i);
      {
        UM982ParserStringify stfy(this, endPos);
        printf("%s\n", &mBuffer[startPos]);
      }
    }
  }
#endif
};

#endif
