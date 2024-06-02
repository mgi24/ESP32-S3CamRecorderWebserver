//ESP32 CAM RECORDER BY MMV PROJECT
//free to distribute but i swear do not remove line one or i will find u
//modified from jamezah junior recorder

//HOW IT WORKS

//Camera Web Server from esp32 example
//ESPAsyncWebserver for virtual button only
//Core 0 for loop camera, core 1 for saving to SD
// IF web button pressed > recording = 1 > start avi > save everyframe > until web stop recording > recording = 0 > end avi

#include "esp_camera.h"
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
AsyncWebServer server(8000);//for second webserver, main server (port80) used by main camera webserver

#include "SD_MMC.h" //SDCARD
//Edit this if you have different pin configuration
#define SDMMC_CLK   39
#define SDMMC_CMD   38
#define SDMMC_D0    40  

//file name increment for avi
int lastfilename;

//initialize SD for the first time / setup
void SD_init(){
  SD_MMC.setPins(SDMMC_CLK, SDMMC_CMD, SDMMC_D0);
  //Set sd speed to 20Mhz, if not set not gonna work in ESP32-s3, try to get it back to 40Mhz if possible
  if(!SD_MMC.begin("/sdcard", true, false, 20000, 5)){
    Serial.println("Card Mount Failed");
    return;
  }
  else{//make sure you have already have tracker.txt on sdcard, with number in it
    File file = SD_MMC.open("/tracker.txt");
    
    if (file) {
      if (file) {
          if (file.available()) {
              lastfilename = file.parseInt();
          }
          file.close();
      } else {
          Serial.println("Gagal membuka file untuk dibaca.");
      }
      Serial.print("Nomor file terakhir ");
      Serial.println(lastfilename);
    }


    uint8_t cardType = SD_MMC.cardType();
    if(cardType == CARD_NONE){
        Serial.println("No SD_MMC card attached");
        return;
    }

    Serial.print("SD_MMC Card Type: ");
    if(cardType == CARD_MMC){
        Serial.println("MMC");
    } else if(cardType == CARD_SD){
        Serial.println("SDSC");
    } else if(cardType == CARD_SDHC){
        Serial.println("SDHC");
    } else {
        Serial.println("UNKNOWN");
    }
    //SD Size
    uint64_t cardSize = SD_MMC.cardSize() / (1024 * 1024);
    Serial.printf("SD_MMC Card Size: %lluMB\n", cardSize);
  }
}

#include "driver/temp_sensor.h" //Temp sensor just to make sure not overheat (my esp32 die)
void initTempSensor(){//temp sensor
    temp_sensor_config_t temp_sensor = TSENS_CONFIG_DEFAULT();
    temp_sensor.dac_offset = TSENS_DAC_L1;  // TSENS_DAC_L2 is default; L4(-40°C ~ 20°C), L2(-10°C ~ 80°C), L1(20°C ~ 100°C), L0(50°C ~ 125°C)
    temp_sensor_set_config(temp_sensor);
    temp_sensor_start();
}

/////////////////////////////////ALL VARIABLE IS HERE////////////////////////////////////////////////
int recording=0;
int framesize;
int quality;
TaskHandle_t Cameraloop;
TaskHandle_t SDloop;
int sd_go=0;
int camera_go=0;
File avifile;
File idxfile;
int frame_cnt = 0;
#define AVIOFFSET 240 // AVI main header length
#define BUFFSIZE 512
uint8_t buf[BUFFSIZE];
int normal_jpg = 0;
int extend_jpg = 0;
int bad_jpg = 0;
camera_fb_t * fb_curr = NULL;
camera_fb_t * fb_next = NULL;
uint16_t remnant = 0;
unsigned long jpeg_size = 0;
unsigned long movi_size = 0;
unsigned long idx_offset = 0;
uint32_t startms;
#define fbs 8 // was 64 -- how many kb of static ram for psram -> sram buffer for sd write
uint8_t framebuffer_static[fbs * 1024 + 20];
struct frameSizeStruct {
  uint8_t frameWidth[2];
  uint8_t frameHeight[2];
};
uint8_t zero_buf[4] = {0x00, 0x00, 0x00, 0x00};
uint8_t dc_buf[4] = {0x30, 0x30, 0x64, 0x63};    // "00dc"
uint8_t dc_and_zero_buf[8] = {0x30, 0x30, 0x64, 0x63, 0x00, 0x00, 0x00, 0x00};

uint8_t avi1_buf[4] = {0x41, 0x56, 0x49, 0x31};    // "AVI1"
uint8_t idx1_buf[4] = {0x69, 0x64, 0x78, 0x31};    // "idx1"
//  data structure from here https://github.com/s60sc/ESP32-CAM_MJPEG2SD/blob/master/avi.cpp, extended for ov5640

static const frameSizeStruct frameSizeData[] = {
  {{0x60, 0x00}, {0x60, 0x00}}, // FRAMESIZE_96X96,    // 96x96
  {{0xA0, 0x00}, {0x78, 0x00}}, // FRAMESIZE_QQVGA,    // 160x120
  {{0xB0, 0x00}, {0x90, 0x00}}, // FRAMESIZE_QCIF,     // 176x144
  {{0xF0, 0x00}, {0xB0, 0x00}}, // FRAMESIZE_HQVGA,    // 240x176
  {{0xF0, 0x00}, {0xF0, 0x00}}, // FRAMESIZE_240X240,  // 240x240
  {{0x40, 0x01}, {0xF0, 0x00}}, // FRAMESIZE_QVGA,     // 320x240   framessize
  {{0x90, 0x01}, {0x28, 0x01}}, // FRAMESIZE_CIF,      // 400x296       bytes per buffer required in psram - quality must be higher number (lower quality) than config quality
  {{0xE0, 0x01}, {0x40, 0x01}}, // FRAMESIZE_HVGA,     // 480x320       low qual  med qual  high quality
  {{0x80, 0x02}, {0xE0, 0x01}}, // FRAMESIZE_VGA,      // 640x480   8   11+   ##  6-10  ##  0-5         indoor(56,COUNT=3)  (56,COUNT=2)          (56,count=1)
                                                       //               38,400    61,440    153,600 
  {{0x20, 0x03}, {0x58, 0x02}}, // FRAMESIZE_SVGA,     // 800x600   9
  {{0x00, 0x04}, {0x00, 0x03}}, // FRAMESIZE_XGA,      // 1024x768  10
  {{0x00, 0x05}, {0xD0, 0x02}}, // FRAMESIZE_HD,       // 1280x720  11  115,200   184,320   460,800     (11)50.000  25.4fps   (11)50.000 12fps    (11)50,000  12.7fps
  {{0x00, 0x05}, {0x00, 0x04}}, // FRAMESIZE_SXGA,     // 1280x1024 12
  {{0x40, 0x06}, {0xB0, 0x04}}, // FRAMESIZE_UXGA,     // 1600x1200 13  240,000   384,000   960,000
  // 3MP Sensors
  {{0x80, 0x07}, {0x38, 0x04}}, // FRAMESIZE_FHD,      // 1920x1080 14  259,200   414,720   1,036,800   (11)210,000 5.91fps
  {{0xD0, 0x02}, {0x00, 0x05}}, // FRAMESIZE_P_HD,     //  720x1280 15
  {{0x60, 0x03}, {0x00, 0x06}}, // FRAMESIZE_P_3MP,    //  864x1536 16
  {{0x00, 0x08}, {0x00, 0x06}}, // FRAMESIZE_QXGA,     // 2048x1536 17  393,216   629,146   1,572,864
  // 5MP Sensors
  {{0x00, 0x0A}, {0xA0, 0x05}}, // FRAMESIZE_QHD,      // 2560x1440 18  460,800   737,280   1,843,200   (11)400,000 3.5fps    (11)330,000 1.95fps
  {{0x00, 0x0A}, {0x40, 0x06}}, // FRAMESIZE_WQXGA,    // 2560x1600 19
  {{0x38, 0x04}, {0x80, 0x07}}, // FRAMESIZE_P_FHD,    // 1080x1920 20
  {{0x00, 0x0A}, {0x80, 0x07}}  // FRAMESIZE_QSXGA,    // 2560x1920 21  614,400   983,040   2,457,600   (15)425,000 3.25fps   (15)382,000 1.7fps  (15)385,000 1.7fps

};

const int avi_header[AVIOFFSET] PROGMEM = {
  0x52, 0x49, 0x46, 0x46, 0xD8, 0x01, 0x0E, 0x00, 0x41, 0x56, 0x49, 0x20, 0x4C, 0x49, 0x53, 0x54,
  0xD0, 0x00, 0x00, 0x00, 0x68, 0x64, 0x72, 0x6C, 0x61, 0x76, 0x69, 0x68, 0x38, 0x00, 0x00, 0x00,
  0xA0, 0x86, 0x01, 0x00, 0x80, 0x66, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00,
  0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x80, 0x02, 0x00, 0x00, 0xe0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4C, 0x49, 0x53, 0x54, 0x84, 0x00, 0x00, 0x00,
  0x73, 0x74, 0x72, 0x6C, 0x73, 0x74, 0x72, 0x68, 0x30, 0x00, 0x00, 0x00, 0x76, 0x69, 0x64, 0x73,
  0x4D, 0x4A, 0x50, 0x47, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x73, 0x74, 0x72, 0x66,
  0x28, 0x00, 0x00, 0x00, 0x28, 0x00, 0x00, 0x00, 0x80, 0x02, 0x00, 0x00, 0xe0, 0x01, 0x00, 0x00,
  0x01, 0x00, 0x18, 0x00, 0x4D, 0x4A, 0x50, 0x47, 0x00, 0x84, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x49, 0x4E, 0x46, 0x4F,
  0x10, 0x00, 0x00, 0x00, 0x6A, 0x61, 0x6D, 0x65, 0x73, 0x7A, 0x61, 0x68, 0x61, 0x72, 0x79, 0x20,
  0x76, 0x35, 0x30, 0x20, 0x4C, 0x49, 0x53, 0x54, 0x00, 0x01, 0x0E, 0x00, 0x6D, 0x6F, 0x76, 0x69,
};

//
// Writes an uint32_t in Big Endian at current file position
//
static void inline print_quartet(unsigned long i, File fd) {

  uint8_t y[4];
  y[0] = i % 0x100;
  y[1] = (i >> 8) % 0x100;
  y[2] = (i >> 16) % 0x100;
  y[3] = (i >> 24) % 0x100;
  size_t i1_err = fd.write(y , 4);
}

//
// Writes 2 uint32_t in Big Endian at current file position
//
static void inline print_2quartet(unsigned long i, unsigned long j, File fd) {

  uint8_t y[8];
  y[0] = i % 0x100;
  y[1] = (i >> 8) % 0x100;
  y[2] = (i >> 16) % 0x100;
  y[3] = (i >> 24) % 0x100;
  y[4] = j % 0x100;
  y[5] = (j >> 8) % 0x100;
  y[6] = (j >> 16) % 0x100;
  y[7] = (j >> 24) % 0x100;
  size_t i1_err = fd.write(y , 8);
}


///////////////////////////////////////////END OF ALL VARIABLE///////////////////////


///////////////////////////////////////////////////////////////START OF ALL TASK/////////////////////////////////////////////

//GET GOOD FRAME

camera_fb_t *  get_good_fb(){
  camera_fb_t * fb;
  int failures = 0;
  do {
    int fblen = 0;
    int foundffd9 = 0;

    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera Capture Failed");
      failures++;
    } else {
      int get_fail = 0;
      fblen = fb->len;
      for (int j = 1; j <= 1025; j++) {
        if (fb->buf[fblen - j] != 0xD9) {
          // no d9, try next for
        } else {                                     //Serial.println("Found a D9");
          if (fb->buf[fblen - j - 1] == 0xFF ) {     //Serial.print("Found the FFD9, junk is "); Serial.println(j);
            if (j == 1) {
              normal_jpg++;
            } else {
              extend_jpg++;
            }
            foundffd9 = 1;
            break;
          }
        }
      }

      if (!foundffd9) {
        bad_jpg++;
        Serial.printf("Bad jpeg, Frame %d, Len = %d \n", frame_cnt, fblen);

        esp_camera_fb_return(fb);
        failures++;
      } else {
        break;
        // count up the useless bytes
      }
    }

  } while (failures < 10);   // normally leave the loop with a break()
  if (failures == 10) {     
    Serial.printf("10 failures");

    sensor_t * ss = esp_camera_sensor_get();
    int qual = ss->status.quality ;
    ss->set_quality(ss, qual + 5);
    quality = qual + 5;
    Serial.printf("\n\nDecreasing quality due to frame failures %d -> %d\n\n", qual, qual + 5);
    delay(1000);
    recording = 0;
    //reboot_now = true;
  }
  return fb;
}
//END OF GET GOOD FRAME




//AVI START
void aviStart(){//injecting header in the begining of file
  
  File file = SD_MMC.open("/tracker.txt", FILE_WRITE);
    
    if (file) {
        file.println(lastfilename);
        file.close();
        Serial.println("Berhasil update nama file.");
    } else {
        Serial.println("Gagal membuka file untuk ditulis.");
    }
  Serial.println("Starting an avi ");
  String avi_file_name = "/"+String(lastfilename)+".avi";

  avifile = SD_MMC.open(avi_file_name, "w");
  idxfile = SD_MMC.open("/idx.tmp", "w");

  Serial.print("avi resolution = ");
  Serial.println(framesize);
  Serial.print("avi quality = ");
  Serial.println(quality);

  if (avifile) {
    Serial.printf("File open: %s\n", avi_file_name);
  }  else  {
    Serial.println("Could not open file");
  }
  if (idxfile)  {
    //Serial.printf("File open: %s\n", "//idx.tmp");
  }  else  {
    Serial.println("Could not open file /idx.tmp");
  }

  for (int i = 0; i < AVIOFFSET; i++){
    char ch = pgm_read_byte(&avi_header[i]);
    buf[i] = ch;
  }

  memcpy(buf + 0x40, frameSizeData[framesize].frameWidth, 2);
  memcpy(buf + 0xA8, frameSizeData[framesize].frameWidth, 2);
  memcpy(buf + 0x44, frameSizeData[framesize].frameHeight, 2);
  memcpy(buf + 0xAC, frameSizeData[framesize].frameHeight, 2);

  size_t err = avifile.write(buf, AVIOFFSET);

  avifile.seek( AVIOFFSET, SeekSet);
  
  Serial.println(F("\nRecording Started"));
  startms = millis();
  bad_jpg = 0;
  extend_jpg = 0;
  normal_jpg = 0;
  jpeg_size = 0;
  movi_size = 0;
  idx_offset = 4;
  avifile.flush();
  
}
// END OF AVISTART

//AVI SAVE FRAME

void another_save_avi(camera_fb_t * fb ){//increment fill the file with fb data
  int fblen;
  fblen = fb->len;

  int fb_block_length;
  uint8_t* fb_block_start;

  jpeg_size = fblen;

  remnant = (4 - (jpeg_size & 0x00000003)) & 0x00000003;

  framebuffer_static[0] = 0x30;       // "00dc"
  framebuffer_static[1] = 0x30;
  framebuffer_static[2] = 0x64;
  framebuffer_static[3] = 0x63;

  int jpeg_size_rem = jpeg_size + remnant;

  framebuffer_static[4] = jpeg_size_rem % 0x100;
  framebuffer_static[5] = (jpeg_size_rem >> 8) % 0x100;
  framebuffer_static[6] = (jpeg_size_rem >> 16) % 0x100;
  framebuffer_static[7] = (jpeg_size_rem >> 24) % 0x100;

  fb_block_start = fb->buf;

  if (fblen > fbs * 1024 - 8 ) {                     // fbs is the size of frame buffer static
    fb_block_length = fbs * 1024;
    fblen = fblen - (fbs * 1024 - 8);
    memcpy(framebuffer_static + 8, fb_block_start, fb_block_length - 8);
    fb_block_start = fb_block_start + fb_block_length - 8;

  } else {
    fb_block_length = fblen + 8  + remnant;
    memcpy(framebuffer_static + 8, fb_block_start,  fblen);
    fblen = 0;
  }

  size_t err = avifile.write(framebuffer_static, fb_block_length);

  if (err != fb_block_length) {
    Serial.print("Error on avi write: err = "); Serial.print(err);
    Serial.print(" len = "); Serial.println(fb_block_length);
  }
  while (fblen > 0) {

    if (fblen > fbs * 1024) {
      fb_block_length = fbs * 1024;
      fblen = fblen - fb_block_length;
    } else {
      fb_block_length = fblen  + remnant;
      fblen = 0;
    }

    memcpy(framebuffer_static, fb_block_start, fb_block_length);

    size_t err = avifile.write(framebuffer_static,  fb_block_length);

    if (err != fb_block_length) {
      Serial.print("Error on avi write: err = "); Serial.print(err);
      Serial.print(" len = "); Serial.println(fb_block_length);
    }

    fb_block_start = fb_block_start + fb_block_length;
    delay(0);
  }

  movi_size += jpeg_size;

  print_2quartet(idx_offset, jpeg_size, idxfile);

  idx_offset = idx_offset + jpeg_size + remnant + 8;

  movi_size = movi_size + remnant;

  avifile.flush();
  Serial.println("SaveFrame Done!");
}
//END OF AVI SAVE FRAME

//START OF END AVI

void end_avi(){//close the file + insert metadata like FPS etc
  unsigned long current_end = avifile.position();

  Serial.println("End of avi - closing the files");

  if (frame_cnt <  5 ) {
    Serial.println("Recording screwed up, less than 5 frames, forget index\n");
    idxfile.close();
    avifile.close();
    String avi_file_name = "/"+String(lastfilename)+".avi";
    int xx = SD_MMC.remove("/idx.tmp");
    int yy = SD_MMC.remove(avi_file_name);

  } else {

    uint32_t elapsedms = millis() - startms;

    float fRealFPS = (1000.0f * (float)frame_cnt) / ((float)elapsedms);

    float fmicroseconds_per_frame = 1000000.0f / fRealFPS;
    uint8_t iAttainedFPS = round(fRealFPS) ;
    uint32_t us_per_frame = round(fmicroseconds_per_frame);

//Modify the MJPEG header from the beginning of the file, overwriting various placeholders

    avifile.seek( 4 , SeekSet);
    print_quartet(movi_size + 240 + 16 * frame_cnt + 8 * frame_cnt, avifile);

    avifile.seek( 0x20 , SeekSet);
    print_quartet(us_per_frame, avifile);

    unsigned long max_bytes_per_sec = (1.0f * movi_size * iAttainedFPS) / frame_cnt;

    avifile.seek( 0x24 , SeekSet);
    print_quartet(max_bytes_per_sec, avifile);

    avifile.seek( 0x30 , SeekSet);
    print_quartet(frame_cnt, avifile);

    avifile.seek( 0x8c , SeekSet);
    print_quartet(frame_cnt, avifile);

    avifile.seek( 0x84 , SeekSet);
    print_quartet((int)iAttainedFPS, avifile);

    avifile.seek( 0xe8 , SeekSet);
    print_quartet(movi_size + frame_cnt * 8 + 4, avifile);
    Serial.println(F("\n*** Video recorded and saved ***\n"));


    Serial.printf("Recorded %5d frames in %5d seconds\n", frame_cnt, elapsedms / 1000);
    Serial.printf("File size is %u bytes\n", movi_size + 12 * frame_cnt + 4);
    Serial.printf("Adjusted FPS is %5.2f\n", fRealFPS);
    Serial.printf("Max data rate is %lu bytes/s\n", max_bytes_per_sec);
    Serial.printf("Frame duration is %d us\n", us_per_frame);
    Serial.printf("Writng the index, %d frames\n", frame_cnt);

    avifile.seek( current_end , SeekSet);

    idxfile.close();

    size_t i1_err = avifile.write(avi1_buf, 4);

    print_quartet(frame_cnt * 16, avifile);

    idxfile = SD_MMC.open("/idx.tmp", "r");
    if (idxfile)  {
      //Serial.printf("File open: %s\n", "//idx.tmp");
      //logfile.printf("File open: %s\n", "/idx.tmp");
    }  else  {
      Serial.println("Could not open index file");
    }

    char * AteBytes;
    AteBytes = (char*) malloc (8);

    for (int i = 0; i < frame_cnt; i++) {
      size_t res = idxfile.readBytes( AteBytes, 8);
      size_t i1_err = avifile.write(dc_buf, 4);
      size_t i2_err = avifile.write(zero_buf, 4);
      size_t i3_err = avifile.write((uint8_t *)AteBytes, 8);
    }
    free(AteBytes);
    idxfile.close();
    avifile.close();
    int xx = remove("/idx.tmp");
  }
}

//END OF END_AVI

////////////////////////////////////////////////////END OF ALL TASK//////////////////////////////////////////////////

////////////////////////////////////////////////// MAIN LOOP START ///////////////////////////////////////////// 
//CORE 0
void cameraloop(void *parameter) {
  Serial.print("CAMERA LOOP using core ");  Serial.print(xPortGetCoreID());
  Serial.print(", priority = "); Serial.println(uxTaskPriorityGet(NULL));
  int isstopped = 0;
  
  while (true) {
    
    if (recording == 0 && frame_cnt ==0){
      if (isstopped ==0){
        Serial.println("NOT RECORDING");
        isstopped=1;
      }
    }//START RECORD
    else if(recording==1 && frame_cnt==0){
      isstopped=0;

      sensor_t * s = esp_camera_sensor_get();
      framesize = s->status.framesize ;
      quality = s->status.framesize ;
      Serial.print("avi resolution = ");
      Serial.print(framesize);
      Serial.print(" avi quality = ");
      Serial.println(quality);

      frame_cnt++;
      
      millis();
      fb_curr = get_good_fb();
      millis();

      aviStart();

      millis();
      fb_next = get_good_fb(); 
      millis();

      sd_go=1;
      
    }//SAVE ANOTHER FRAME
    else if(recording == 1 && frame_cnt>0){

      if(camera_go==1){
        camera_go=0;
        frame_cnt++;

        esp_camera_fb_return(fb_curr);

        fb_curr = fb_next;

        sd_go=1;
        fb_next = get_good_fb();  
      }

    }//END RECORD
    
    else if(recording==0&& frame_cnt>0){
      if (camera_go==1){
        camera_go=0;        

        esp_camera_fb_return(fb_curr);

        frame_cnt++;
        fb_curr = fb_next;
        fb_next = NULL;

        another_save_avi(fb_curr);

        esp_camera_fb_return(fb_curr);
        fb_curr = NULL;

        end_avi();
        
        float fps = 1.0 * frame_cnt / ((millis() - startms) / 1000) ;
        Serial.print("FPS: ");
        Serial.println(fps);

        frame_cnt = 0;
        Serial.println("End the Avi");
      }

    }
    vTaskDelay(portTICK_PERIOD_MS);
  }
}

// CORE 1
void sdloop(void *parameter) {

  Serial.print("SD LOOP using core ");  Serial.print(xPortGetCoreID());
  Serial.print(", priority = "); Serial.println(uxTaskPriorityGet(NULL));

  while (true) {
    if(sd_go==1){//i use int because i failed to use xsemaphor
      sd_go=0;
      another_save_avi(fb_curr);                        // do the actual sd wrte
      camera_go=1;
    }
    vTaskDelay(portTICK_PERIOD_MS);
  }
}


////////////////////////////////////////////////// MAIN LOOP END  ///////////////////////////////////////////// 



//BELOW FROM ESP CAMERA WEBSERVER
//
// WARNING!!! PSRAM IC required for UXGA resolution and high JPEG quality
//            Ensure ESP32 Wrover Module or other board with PSRAM is selected
//            Partial images will be transmitted if image exceeds buffer size
//
//            You must select partition scheme from the board menu that has at least 3MB APP space.
//            Face Recognition is DISABLED for ESP32 and ESP32-S2, because it takes up from 15 
//            seconds to process single frame. Face Detection is ENABLED if PSRAM is enabled as well

// ===================
// Select camera model
// ===================
//#define CAMERA_MODEL_WROVER_KIT // Has PSRAM
//#define CAMERA_MODEL_ESP_EYE // Has PSRAM
#define CAMERA_MODEL_ESP32S3_EYE // Has PSRAM
//#define CAMERA_MODEL_M5STACK_PSRAM // Has PSRAM
//#define CAMERA_MODEL_M5STACK_V2_PSRAM // M5Camera version B Has PSRAM
//#define CAMERA_MODEL_M5STACK_WIDE // Has PSRAM
//#define CAMERA_MODEL_M5STACK_ESP32CAM // No PSRAM
//#define CAMERA_MODEL_M5STACK_UNITCAM // No PSRAM
//#define CAMERA_MODEL_AI_THINKER // Has PSRAM
//#define CAMERA_MODEL_TTGO_T_JOURNAL // No PSRAM
//#define CAMERA_MODEL_XIAO_ESP32S3 // Has PSRAM
// ** Espressif Internal Boards **
//#define CAMERA_MODEL_ESP32_CAM_BOARD
//#define CAMERA_MODEL_ESP32S2_CAM_BOARD
//#define CAMERA_MODEL_ESP32S3_CAM_LCD
//#define CAMERA_MODEL_DFRobot_FireBeetle2_ESP32S3 // Has PSRAM
//#define CAMERA_MODEL_DFRobot_Romeo_ESP32S3 // Has PSRAM
#include "camera_pins.h"


// ===========================
// Enter your WiFi credentials
// ===========================
const char* ssid = "realme";
const char* password = "12345678";

void startCameraServer();
void setupLedFlash(int pin);




////////////////////////////////////////////////////////////START OF SETUP////////////////////////////////////////////////
void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();
  initTempSensor();
  SD_init();
  Serial.print("setup using core ");  Serial.print(xPortGetCoreID());
  Serial.print(", priority = "); Serial.println(uxTaskPriorityGet(NULL));

  ///////////////////////START OF default esp camera/////////////////////
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_UXGA;
  config.pixel_format = PIXFORMAT_JPEG; // for streaming
  //config.pixel_format = PIXFORMAT_RGB565; // for face detection/recognition
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;
  
  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  //                      for larger pre-allocated frame buffer.
  if(config.pixel_format == PIXFORMAT_JPEG){
    if(psramFound()){
      config.jpeg_quality = 10;
      config.fb_count = 2;
      config.grab_mode = CAMERA_GRAB_LATEST;
    } else {
      // Limit the frame size when PSRAM is not available
      config.frame_size = FRAMESIZE_SVGA;
      config.fb_location = CAMERA_FB_IN_DRAM;
    }
  } else {
    // Best option for face detection/recognition
    config.frame_size = FRAMESIZE_240X240;
#if CONFIG_IDF_TARGET_ESP32S3
    config.fb_count = 2;
#endif
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1); // flip it back
    s->set_brightness(s, 1); // up the brightness just a bit
    s->set_saturation(s, -2); // lower the saturation
  }
  // drop down frame size for higher initial frame rate
  if(config.pixel_format == PIXFORMAT_JPEG){
    s->set_framesize(s, FRAMESIZE_QVGA);
  }

#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

#if defined(CAMERA_MODEL_ESP32S3_EYE)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);//MINE IS FLIPPED FOR SOME REASON?
#endif

// Setup LED FLash if LED pin is defined in camera_pins.h
#if defined(LED_GPIO_NUM)
  setupLedFlash(LED_GPIO_NUM);
#endif

  WiFi.begin(ssid, password);
  WiFi.setSleep(false);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  //website that work on port 80
  startCameraServer();
  
  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");

  //////////////////////////////////////END OF DEFAULT CAMERA/////////////////////////
  //website that work on port 8000
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    String html = "<!DOCTYPE html><html lang=\"en\">";
    html += "<head><meta charset=\"UTF-8\"><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">";
    html += "<title>Nomor</title></head>";
    html += "<body><h1>Nomor file: " + String(lastfilename) + "</h1>";
    html += "<p>Status: Not Recording </p>";
    html += "<form action=\"/start\" method=\"get\"><button type=\"submit\">Start Record</button></form>";
    html += "</body></html>";
    recording = 0;
    request->send(200, "text/html", html);
  });

  // Menangani URL "/start"
  server.on("/start", HTTP_GET, [](AsyncWebServerRequest *request) {
    recording = 1;
    lastfilename++;
    request->send_P(200, "text/html", R"rawliteral(
      <!DOCTYPE HTML><html>
      <head>
        <title>ESP32 Web Server</title>
        <meta name="viewport" content="width=device-width, initial-scale=1">
      </head>
      <body>
        <h1>ESP32 Web Server</h1>
        <p>Recording...</p>
        <p><button onclick="stopRecording()">Stop Recording</button></p>
        <script>
          function stopRecording() {
            window.location.href = "/";
          }
        </script>
      </body>
      </html>
    )rawliteral");
  });
  //split workload to 2 core 
    xTaskCreatePinnedToCore(
    cameraloop,     // Fungsi untuk task1
    "Cameraloop",   // Nama task1
    10000,     // Ukuran stack
    NULL,      // Parameter task1
    1,         // Prioritas task1
    &Cameraloop,    // Handle task1
    0          // Jalankan pada core 0
  );
  delay(200);
    // Menjalankan task2 pada core 1
  xTaskCreatePinnedToCore(
    sdloop,     // Fungsi untuk task2
    "SDloop",   // Nama task2
    10000,     // Ukuran stack
    NULL,      // Parameter task2
    2,         // Prioritas task2
    &SDloop,    // Handle task2
    1          // Jalankan pada core 1
  );

  server.begin();
  framesize = s->status.framesize ;
  quality = s->status.framesize ;
  delay(100);
  Serial.print("DEFAULT CAMERA RESOLUTION: ");
  Serial.println(framesize);
  Serial.print("DEFAULT CAMERA QUALITY: ");
  Serial.println(quality);
}

///////////////////////////////////////////////END OF SETUP////////////////////////////////////////

void loop() {//not main loop!!! main loop already run on core 0 and 1
  Serial.print("Suhu core: ");
  float coretemp = 0;
  temp_sensor_read_celsius(&coretemp);
  Serial.println(coretemp);
  delay(10000);
}
