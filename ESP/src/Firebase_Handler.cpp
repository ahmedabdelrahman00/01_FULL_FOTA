#include "Firebase_Handler.h"
#include "Debug.h"

#if defined(ESP32)
#include <WiFi.h>
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#endif
#include <Firebase_ESP_Client.h>
//Provide the token generation process info.
#include <addons/TokenHelper.h>
#include <addons/RTDBHelper.h>

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;
FirebaseJson json;

int Global_AppServerVersion;
int Global_AppCarVersion;

bool taskCompleted = false;
unsigned long sendDataPrevMillis = 0;
int Version_int;
bool signupOK = false;

static void fcsDownloadCallback(FCS_DownloadStatusInfo info);

void Wifi_Connect(void)
{
  Serial.begin(BAUD_RATE);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  debug("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED)
  {
    debug(".");
    delay(300);
  }
  debugln();
  debug("Connected with IP: ");
  debugln(WiFi.localIP());
  debugln();
}

void Server_Connect(void){
  debugf("Firebase Client v%s\n\n", FIREBASE_CLIENT_VERSION);

  /* Assign the api key (required) */
  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;

  /* Assign the user sign in credentials */
  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;

  if (Firebase.signUp(&config, &auth, "", ""))
  {
    // Serial.println("ok");
    signupOK = true;
  }

  else {
    Serial.printf("%s\n", config.signer.signupError.message.c_str());
  }
  /* Assign the callback function for the long running token generation task */
  config.token_status_callback = tokenStatusCallback; //see addons/TokenHelper.h

  #if defined(ESP8266)
  //required for large file data, increase Rx size as needed.
  fbdo.setBSSLBufferSize(1024 /* Rx buffer size in bytes from 512 - 16384 */, 1024 /* Tx buffer size in bytes from 512 - 16384 */);
  #endif

  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
}

void Server_Download(const char* file){
  char downloadingStatus = 0;
  char fileRemovingStatus = 0;
  bool Once = false;
  if (Firebase.ready())
  {
     if (!LittleFS.begin()) {
       debugln("Failed to mount LittleFS");
     }
     debugln("LittleFS mounted successfully");
     /* Remove the old file */
     if ((LittleFS.remove("/fileupdate.bin"))){
         debugln("Old file removed successfully!");
     }
     else{
         debugln("Failed to remove old file.");
     } 
    /* Download the new file */
    debugln("Download Status:");
    do {
          if(Once){
            debug("Downloading Faild due to: ");
            Serial.println(fbdo.errorReason()); 
            debugln("Retrying after 5 Seconds!");
            delay(5000);
            debugln("Retrying download now ...");
          }
          downloadingStatus = 
                       Firebase.Storage.download(&fbdo, 
                                                 STORAGE_BUCKET_ID /* Firebase Storage bucket id */, 
                                                 file /* path of remote file stored in the bucket */, 
                                                 "/fileupdate.bin" /* path to local file */, 
                                                 mem_storage_type_flash /* memory storage type, mem_storage_type_flash and mem_storage_type_sd */, 
                                                 fcsDownloadCallback /* callback function */);
          Once = true;                                       
       } while(!downloadingStatus);    
  }
  else
    debugln("--> Firebase is not Ready Right Now");
}

static void fcsDownloadCallback(FCS_DownloadStatusInfo info){
    if (info.status == firebase_fcs_download_status_init)
    {
        Serial.printf("Downloading file %s (%d) to %s\n", info.remoteFileName.c_str(), info.fileSize, info.localFileName.c_str());
    }
    else if (info.status == firebase_fcs_download_status_download)
    {
        Serial.printf("Downloaded %d%s, Elapsed time %d ms\n", (int)info.progress, "%", info.elapsedTime);
    }
    else if (info.status == firebase_fcs_download_status_complete)
    {
        Serial.println("Download completed\n");
    }
    else if (info.status == firebase_fcs_download_status_error)
    {
        Serial.printf("Download failed, %s\n", info.errorMsg.c_str());
    }
}

int Version_Recieve(void){
  if (Firebase.ready())
  {
    if (Firebase.RTDB.getInt(&fbdo, "/version"))
    {
      if (fbdo.dataType() == "int")
      {
        Version_int = fbdo.intData();
        Serial.print("Function Version = ");
        Serial.println(Version_int);
        return Version_int;
      }
      else
        return 0;
    }
    else {
      Serial.println(fbdo.errorReason());
        return 0;
    }
  }
   else
   {
    return 0;
   }
}

bool UpdateCheck(void){
  Global_AppServerVersion = Version_Recieve();
  /* New Update Available */
  if (Global_AppCarVersion < Global_AppServerVersion){
      Global_AppCarVersion = Global_AppServerVersion;
      debugln("--> Update Available");
      return true;	
  }
  /* No Update Available */
  else{
      debugln("No Update Available, Right Now");
      return false;
  }
}
