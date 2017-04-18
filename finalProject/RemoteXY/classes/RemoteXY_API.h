#ifndef _REMOTEXY_API_H_
#define _REMOTEXY_API_H_

#include <inttypes.h> 
#include <stdlib.h>
#include <Arduino.h>


#define REMOTEXY_PACKAGE_START_BYTE 0x55
#define REMOTEXY_PASSWORD_LENGTH_MAX 26
#define REMOTEXY_TIMEOUT 5000
#define REMOTEXY_SERVER_TIMEOUT 7000
#define REMOTEXY_CLOUD_CONNECT_TIMEOUT 10000
#define REMOTEXY_CLOUD_RECONNECT_TIMEOUT 30000
#define REMOTEXY_CLOUD_ECHO_TIMEOUT 3900000   // 6.5 min, server echo = 3 min


class CRemoteXY_API {
  protected:
  uint8_t confVersion;
  uint8_t *conf;
  uint8_t *var;
  uint8_t *accessPassword;
  uint16_t outputLength;
  uint16_t inputLength;
  uint16_t confLength;
  uint8_t *connect_flag;

  uint8_t *receiveBuffer;
  uint16_t receiveBufferLength;
  uint16_t receiveIndex;
  uint16_t receiveCRC;

  uint32_t wireTimeOut;
  
  uint8_t moduleRunning;
    
  protected:
  virtual uint8_t initModule () {return 1;};
  virtual void handlerModule () {};
  virtual void closeConnection () {};
  virtual void sendStart (uint16_t len) {};
  virtual void sendByte (uint8_t b) {};
  virtual uint8_t receiveByte () {};
  virtual uint8_t availableByte () {};  
  
  public:
  void init (const void * _conf, void * _var, const char * _accessPassword) {
    uint32_t ms;
    uint8_t i;
    uint8_t* p = (uint8_t*)_conf;
    uint8_t b = getConfByte (p++);
    if (b==0xff) {
      inputLength = getConfByte (p++);
      inputLength |= getConfByte (p++)<<8;
      outputLength = getConfByte (p++);
      outputLength |= getConfByte (p++)<<8;
    }
    else {
      inputLength = b;
      outputLength = getConfByte (p++);    
    }
    confLength = getConfByte (p++);
    confLength |= getConfByte (p++)<<8;
    conf = p;
    confVersion = getConfByte (p);
    var = (uint8_t*)_var;
    uint16_t varLength = outputLength+inputLength;
    connect_flag = var+varLength;
    *connect_flag = 0;   
        
    accessPassword = (uint8_t*)_accessPassword;

    receiveBufferLength=inputLength;
    if ((*accessPassword!=0)&&(receiveBufferLength<REMOTEXY_PASSWORD_LENGTH_MAX))
      receiveBufferLength = REMOTEXY_PASSWORD_LENGTH_MAX;
    receiveBufferLength +=6;  
    
    receiveBuffer = (uint8_t*)malloc (receiveBufferLength);             
    
    p = var;
    i = varLength;
    while (i--) *p++=0;   
    receiveIndex=0;
    receiveCRC=initCRC ();
    wireTimeOut=0;
  
#if defined(REMOTEXY__DEBUGLOGS)
    REMOTEXY__DEBUGLOGS.begin (REMOTEXY__DEBUGLOGS_SPEED);
    REMOTEXY__DEBUGLOGS.println("\r\n\r\nRemoteXY started...");
#endif
  
    moduleRunning = initModule ();
#if defined(REMOTEXY__DEBUGLOGS)
    if (moduleRunning) {
      REMOTEXY__DEBUGLOGS.println("\r\nRemoteXY runing");
    }
    else {
      REMOTEXY__DEBUGLOGS.println ("\r\nModule not available, RemoteXY stoped");
    }
    
#endif     
    
    
#if defined(REMOTEXY__DEBUGLOGS)
    REMOTEXY__DEBUGLOGS.print("\r\n<- ");  
#endif
  }

  
  private:
  inline uint8_t getConfByte (uint8_t* p) {
    return pgm_read_byte_near (p);                                     
  }
  
    
  public:
  void handler () {
    uint8_t *p, *kp;
    uint16_t i;  
    uint8_t b;
    uint16_t packageLength;
    
    if (!moduleRunning) return;
    
    handlerModule ();
    
#if defined(REMOTEXY_CLOUD)  
    handlerCloud ();
#endif
    
    while (availableByte () > 0) {  
      b = receiveByte ();  
      
      if ((receiveIndex==0) && (b!=REMOTEXY_PACKAGE_START_BYTE)) continue;   
      receiveBuffer[receiveIndex++]=b;
      updateCRC (&receiveCRC, b);
      if (receiveIndex>receiveBufferLength) {       
        searchStartByte (1); //receiveBuffer overflow
      }
      while (true) {
        if (receiveIndex<6) return;
        packageLength = receiveBuffer[1]|(receiveBuffer[2]>>8);
        if (packageLength>receiveBufferLength) searchStartByte (1); // error
        else if (packageLength<6) searchStartByte (1); // error
        else if (packageLength==receiveIndex) {      
          if (receiveCRC==0) {
            if (handleReceivePackage ()) {
              receiveIndex=0;
              receiveCRC=initCRC ();
              break;
            }
          }
          searchStartByte (1); // error 
        }
        else if (packageLength<receiveIndex) {
          uint16_t crc = initCRC ();
          p = receiveBuffer;
          i=packageLength;
          while (i--) updateCRC (&crc, *(p++)); 
          if (crc==0) {
            if (handleReceivePackage ()) {
              searchStartByte (packageLength);
              continue;
            }
          }
          searchStartByte (1);        
        }
        else break;
      }
    }  
    
    if (wireTimeOut<millis()) {
      *connect_flag = 0;
    }      
  }
  
  private:
  uint16_t initCRC () {
    return 0xffff;
  }          
    
  private:
  void updateCRC (uint16_t *crc, uint8_t b) {
    *crc ^= b;
    for (uint8_t i=0; i<8; ++i) {
      if ((*crc) & 1) *crc = ((*crc) >> 1) ^ 0xA001;
      else *crc >>= 1;
    }
  } 
  
  private:  
  void sendByteUpdateCRC (uint8_t b, uint16_t *crc) {
    sendByte (b); 
    updateCRC (crc, b);
  }  

  private:  
  void sendPackage (uint8_t command, uint8_t *p, uint16_t length, uint8_t itConf) {
    uint16_t crc = initCRC ();
    uint16_t packageLength = length+6;
    sendStart (packageLength);
#if defined(REMOTEXY__DEBUGLOGS)
    REMOTEXY__DEBUGLOGS.print("\r\n-> ");  
#endif
    sendByteUpdateCRC (REMOTEXY_PACKAGE_START_BYTE, &crc);
    sendByteUpdateCRC (packageLength, &crc);
    sendByteUpdateCRC (packageLength>>8, &crc);
    sendByteUpdateCRC (command, &crc);  
    uint8_t b;
    while (length--) {
      if (itConf) b=getConfByte (p++);
      else b=*p++;
      sendByteUpdateCRC (b, &crc);
    }
    sendByte (crc);  
    sendByte (crc>>8);
#if defined(REMOTEXY__DEBUGLOGS)
    REMOTEXY__DEBUGLOGS.print("\r\n<- ");  
#endif 
  }
  
  private:  
  void searchStartByte (uint16_t pos) {
    uint8_t *p, *kp;
    uint16_t i;
    uint16_t ri = receiveIndex;
    p=receiveBuffer+pos;
    receiveCRC=initCRC ();
    for (i=pos; i<ri; i++) {
      if (*p==REMOTEXY_PACKAGE_START_BYTE) {      
        kp=receiveBuffer;
        receiveIndex=receiveIndex-i;
        while (i++<ri) {
          updateCRC (&receiveCRC, *p);              
          *(kp++)=*(p++);
        }
        return;
      }
      p++;
    }        
    receiveIndex=0;
  }  
  
  uint8_t handleReceivePackage () {
    uint8_t command, i;
    uint16_t length;
    uint8_t *p, *kp;
       
    length = receiveBuffer[1]|(receiveBuffer[2]>>8); 
    length-=6;
    command = receiveBuffer[3];
    switch (command) {  
      case 0x00:  
        uint8_t available;
        if (length==0) {
          if (*accessPassword==0) available=1;
          else available=0;
        }
        else {
          uint8_t ch;
          available = 1;
          p = receiveBuffer+4;
          kp = accessPassword; 
          while (true) {
            ch=*kp++;
            if (ch!=*p++) available=0;
            if (!ch) break;
          }                               
        } 
        if (available!=0) {
          sendPackage (command, conf, confLength,  1);
          *connect_flag = 1;
        }
        else {
          uint8_t buf[4];
          p = buf;
          kp = conf;         
          i=confVersion>=5?3:2;
          length = i+1;
          while (i--) *p++ = getConfByte(kp++);
          *p++ = 0xf0;
          sendPackage (command, buf, length,  0);
        }          
        break;   
      case 0x40:  
        sendPackage (command, var, inputLength+outputLength, 0); 
        break;   
      case 0x80:  
        if (length==inputLength) {
          p=receiveBuffer+4;
          kp=var;
          i= inputLength;
          while (i--) *kp++=*p++;
        }
        sendPackage (command, 0, 0, 0);
        break;   
      case 0xC0:  
        sendPackage (command, var+inputLength, outputLength, 0);
        break;   
#if defined(REMOTEXY_CLOUD)  
      case 0x10: // echo
        sendPackage (command, 0, 0, 0);
        break;   
      case 0x11:
        if (cloudState==2) {
          cloudState = 3;
#if defined(REMOTEXY__DEBUGLOGS)
          REMOTEXY__DEBUGLOGS.println("\r\nCloud server registred");
          REMOTEXY__DEBUGLOGS.print("\r\n<- ");  
#endif
        }
        break;   
#endif //REMOTEXY_CLOUD       
      default:
        return 0; 
    }  
    wireTimeOut=millis()+REMOTEXY_TIMEOUT;
#if defined(REMOTEXY_CLOUD)  
    if (cloudState==3) cloudTimeOut=millis()+REMOTEXY_CLOUD_ECHO_TIMEOUT;
#endif //REMOTEXY_CLOUD       
    return 1;
  }
  

#if defined(REMOTEXY_CLOUD)  
// CLOUD SUPPORT
  protected:
  char *cloudServer;
  uint16_t cloudPort;
  uint8_t cloudRegistPackage[38];
  uint8_t cloudState;   // 0 - disconnect, 1 - server connected, 2 - cloud registring, 3 - cloud registered
  uint32_t cloudTimeOut;


  virtual int8_t connectServerCloud (char * _cloudServer, uint16_t _cloudPort) {return 0;};

  public:
  void initCloud (const char * _cloudServer, uint16_t _cloudPort, const char * _cloudToken) {
    cloudServer = (char *) _cloudServer;
    cloudPort = _cloudPort;
    
    uint8_t i;
    uint8_t *p = cloudRegistPackage;
    *p++ = getConfByte(conf+0);
    *p++ = 0;    
    for (i=0; i<32; i++) {
      if (*_cloudToken==0) *(p++)=0;
      else *(p++)=*(_cloudToken++);
    }
    uint16_t *len = (uint16_t*)p;
    *len = outputLength + inputLength;
    if (confLength>*len) *len = confLength;   
    *len += 6+1;    
    len = (uint16_t*)(p+2);     
    *len = receiveBufferLength;  
        
    cloudState = 0;
    cloudTimeOut = 0;
  }
  
  public:
  void startCloudConnection () {
    if (cloudState<=1) cloudTimeOut = 0;  
  }  
  
  public:
  void stopCloud () {
    if (cloudState<=1) return;
    cloudState = 0;
    cloudTimeOut = millis()+REMOTEXY_CLOUD_RECONNECT_TIMEOUT;
#if defined(REMOTEXY__DEBUGLOGS)
    REMOTEXY__DEBUGLOGS.println("\r\nCloud server disconnected");
#endif
  }
  
  
  private:
  void handlerCloud () {
    int8_t r;
    if (!moduleRunning) return;
    if (cloudState == 0) {
      if (cloudTimeOut<=millis()) {
        r = connectServerCloud (cloudServer, cloudPort);
        if (r == 1) {
          cloudState = 1;
#if defined(REMOTEXY__DEBUGLOGS)
          REMOTEXY__DEBUGLOGS.println();
          REMOTEXY__DEBUGLOGS.println("Cloud server connected");
#endif       
        }
        else if (r == 0) {
          cloudTimeOut = millis()+REMOTEXY_CLOUD_RECONNECT_TIMEOUT;
#if defined(REMOTEXY__DEBUGLOGS)
          REMOTEXY__DEBUGLOGS.println();
          REMOTEXY__DEBUGLOGS.println("Cloud server connection error");
#endif         
        }
        else {
          cloudTimeOut = millis()+500;
        }
      }
    }
    else if (cloudState == 1) {
      sendPackage (0x11, cloudRegistPackage, 38, 0);
      cloudTimeOut = millis()+ REMOTEXY_CLOUD_CONNECT_TIMEOUT; 
      cloudState = 2;   
    }
    else if ((cloudState == 2) || (cloudState == 3)) {
      if (cloudTimeOut<=millis()) {
        closeConnection ();
        stopCloud ();
      }
    }
  } 
  
  
  
#endif //REMOTEXY_CLOUD

    
};

#endif //_REMOTEXY_API_H_

