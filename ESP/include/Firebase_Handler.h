#ifndef SERVER_H_
#define SERVER_H_

#include <Arduino.h>

//Define the WiFi credentials
#define WIFI_SSID             "Ahmeeeed_01"  /* Mi Hunters 74 */
#define WIFI_PASSWORD         "A369540020a$@Oo" /* eslam2000 _Hunter_007 */
//Define the API Key
#define API_KEY               "AIzaSyADCXJAtMSX0Fi8jtgz90yF-DepnijSogE"
#define DATABASE_URL          "https://full-fota-default-rtdb.firebaseio.com/"
 
//Define the user Email and password that alreadey registerd or added in your project
#define USER_EMAIL            "ebrahem.mostafa.1812@gmail.com"

#define USER_PASSWORD         "123456789"

//Define the Firebase storage bucket ID e.g bucket-name.appspot.com 
#define STORAGE_BUCKET_ID     "full-fota.appspot.com" 
#define BAUD_RATE              9600
#define RXD2                   16
#define TXD2                   17

void Wifi_Connect     (void);
void Server_Connect   (void);
void Server_Download  (const char* file);
int  Version_Recieve  (void); 
bool UpdateCheck      (void);

#endif