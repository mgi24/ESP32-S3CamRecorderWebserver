//ESP32 Simple Camera Recorder to SD by MMV Project
//modified from jamezah junior recorder
//Make sure to take care of temp problem, esp32 camera can get very hot and destroy itself without sufficient cooling!!!
//i disabled core temp monitoring because it was legacy feature and will be removed in the future
//THIS SKETCH IS TESTED ON ESP32 BOARD MANAGER V2.0.12, FUTURE UPDATE MIGHT BROKE IT!!!
#include "esp_camera.h"

#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
AsyncWebServer server(8000); //second webserver on port 8000

#include "SD_MMC.h" //SDCARD
//Edit this if you have different pin configuration
#define SDMMC_CLK   39
#define SDMMC_CMD   38
#define SDMMC_D0    40 
#define SDMMC_D1    41 
#define SDMMC_D2    42 
#define SDMMC_D3    1

bool is_mic = true; //EDIT TO FALSE IF U ARE NOT USING i2S MIC 

#include "driver/i2s.h"//MIC ignore if u don't use it
#define I2S_MIC_SERIAL_CLOCK 21
#define I2S_MIC_LEFT_RIGHT_CLOCK 47
#define I2S_MIC_SERIAL_DATA 14

#define SAMPLE_BUFFER_SIZE 1024
#define SAMPLE_RATE 48000
#define BITRATE 16
#define CHANNEL 1;

//pin for manual start / stop recording & long press take picture!
#define CAPTURE_BTN 2

//pin for indicator
#define MAIN_LED LED_BUILTIN

//make sure the bit/sample is correct!
i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = 1024,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0};

i2s_pin_config_t i2s_mic_pins = {
    .bck_io_num = I2S_MIC_SERIAL_CLOCK,
    .ws_io_num = I2S_MIC_LEFT_RIGHT_CLOCK,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_MIC_SERIAL_DATA};

///////////////////////////////////// ALL VARIABLE IS HERE ////////////////////////////////////////////////
String APssid = "ESP 32 Camera";
String APpassword = "";

//start/stop the recording
int recording=0;
//holding camera data
int framesize;
int quality;
int frame_cnt = 0;
float fps;

//sdcard failed notifier
bool sdfail = 0;

//file name increment for avi
int lastfilename;

//handle task
TaskHandle_t Cameraloop;
TaskHandle_t SDloop;
TaskHandle_t Audioloop;
TaskHandle_t Blinkloop;

//semaphore resource
int sd_go=0;//saving frame to SD
int camera_go=0;//taking frame buffer
bool audiorecord = false;//takind i2s buffer

bool i2s_go=true;//saving i2s buff to sd

//files
File avifile;
File idxfile;
File wavfile;
File photofile;

//audio variable
int16_t audiobuff[SAMPLE_BUFFER_SIZE];//for taking i2s buffer
int16_t bigbuff[SAMPLE_BUFFER_SIZE*10];// save buffer audio on bigger buffer, then save it to sd at once!
int audiobuffleng=0;

#define AVIOFFSET 240 // AVI main header length
#define BUFFSIZE 512 //for injecting avi header
uint8_t buf[BUFFSIZE];

//log
int normal_jpg = 0;
int extend_jpg = 0;
int bad_jpg = 0;

//frame buffer
camera_fb_t * fb_curr = NULL;
camera_fb_t * fb_next = NULL;
uint16_t remnant = 0;
unsigned long jpeg_size = 0;
unsigned long movi_size = 0;
unsigned long idx_offset = 0;

//timer
uint32_t startms;

//SOME BUFFER

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


//AVI RESOLUTION SETTINGS
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


/////////////////////////////////////////// END OF ALL VARIABLE ///////////////////////////////


/////////////////////////////////////////// ALL TASK ///////////////////////////////////////////

// Writes an uint32_t in Big Endian at current file position
static void inline print_quartet(unsigned long i, File fd) {

  uint8_t y[4];
  y[0] = i % 0x100;
  y[1] = (i >> 8) % 0x100;
  y[2] = (i >> 16) % 0x100;
  y[3] = (i >> 24) % 0x100;
  size_t i1_err = fd.write(y , 4);
}
// Writes 2 uint32_t in Big Endian at current file position
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

//WAV HEADER
void CreateWavHeader(byte* header, int waveDataSize){
  uint16_t channels = CHANNEL;
  uint32_t sampleRate = SAMPLE_RATE;
  uint16_t bitsPerSample = BITRATE;
  uint32_t byteRate = sampleRate * channels * bitsPerSample / 8;
  header[0] = 'R';
  header[1] = 'I';
  header[2] = 'F';
  header[3] = 'F';
  unsigned int fileSizeMinus8 = waveDataSize + 44 - 8;
  header[4] = (byte)(fileSizeMinus8 & 0xFF);
  header[5] = (byte)((fileSizeMinus8 >> 8) & 0xFF);
  header[6] = (byte)((fileSizeMinus8 >> 16) & 0xFF);
  header[7] = (byte)((fileSizeMinus8 >> 24) & 0xFF);
  header[8] = 'W';
  header[9] = 'A';
  header[10] = 'V';
  header[11] = 'E';
  header[12] = 'f';
  header[13] = 'm';
  header[14] = 't';
  header[15] = ' ';
  header[16] = 0x10;  // linear PCM
  header[17] = 0x00;
  header[18] = 0x00;
  header[19] = 0x00;
  header[20] = 0x01;  // linear PCM
  header[21] = 0x00;
  header[22] = 0x01;  // monoral
  header[23] = 0x00;
  header[24] = (sampleRate & 0xFF); //sample rate
  header[25] = ((sampleRate >> 8) & 0xFF);
  header[26] = ((sampleRate >> 16) & 0xFF);
  header[27] = ((sampleRate >> 24) & 0xFF);
  header[28] = (byteRate & 0xFF); //byte rate
  header[29] = ((byteRate >> 8) & 0xFF);
  header[30] = ((byteRate >> 16) & 0xFF);
  header[31] = ((byteRate >> 24) & 0xFF);
  header[32] = 0x02;  // 16bit monoral
  header[33] = 0x00;
  header[34] = 0x10;  // 16bit
  header[35] = 0x00;
  header[36] = 'd';
  header[37] = 'a';
  header[38] = 't';
  header[39] = 'a';
  header[40] = (byte)(waveDataSize & 0xFF);
  header[41] = (byte)((waveDataSize >> 8) & 0xFF);
  header[42] = (byte)((waveDataSize >> 16) & 0xFF);
  header[43] = (byte)((waveDataSize >> 24) & 0xFF);
}
void incrementtracker(){
  lastfilename++;
  File file = SD_MMC.open("/tracker.txt", FILE_WRITE);
  if (file) {
      file.println(lastfilename);
      file.close();
      Serial.println("Tracker updated!");
  } else {
      Serial.println("Gagal membuka file untuk ditulis.");
  }
}


//MAKE SURE ITS GOOD FRAME BUFFER
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
    recording=0; //cancel recording, usually because SD not inserted!!!
    sdfail = 1;
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
  //Serial.println("SaveFrame Done!");
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

    //REMOVE CODE BELOW UNTIL END OF AVI IF U DON'T USE AUDIO!!!
    
    if (is_mic){
      int waveDataSize = int (elapsedms / 1000) * SAMPLE_RATE * BITRATE * 1 / 8;
      wavfile.seek(0);//back to start of the file to write header
      int headerSize = 44;
      byte header[headerSize];
      CreateWavHeader(header, waveDataSize);
      wavfile.write(header, headerSize);
      Serial.println("Mic Finish");
      wavfile.close();
    }
  }
}

//END OF END_AVI

//TAKE PICTURE ONLY
void takepic(){
  sensor_t *s = esp_camera_sensor_get();
  framesize = s->status.framesize ;
  quality = s->status.quality ;
  s->set_framesize(s, (framesize_t)13);//max res UXGA
  s->set_quality(s, 4);//max quality
  camera_fb_t * fb = NULL;
  digitalWrite(MAIN_LED, HIGH);
  delay(1000);
  fb = esp_camera_fb_get();
  digitalWrite(MAIN_LED, LOW);
  if(!fb) {
    Serial.println("Camera capture failed");
    return;
  }
  photofile = SD_MMC.open("/"+String(lastfilename)+".jpg", "w");
  photofile.write(fb->buf, fb->len);
  photofile.close();
  esp_camera_fb_return(fb); 
  s->set_framesize(s, (framesize_t)framesize);//back to original setting
  s->set_quality(s, quality);
}



/////////////////////////////////////////////// END OF ALL TASK //////////////////////////////////////////////////


////////////////////////////////////////////////// MAIN LOOP START ///////////////////////////////////////////// 

//MAIN LOOP
long buttontimer;
bool is_btn=0;//did already pressed before?

void buttonloop(){
  if (analogRead(CAPTURE_BTN) <1000){
    buttontimer = millis();
    is_btn=1;
    while(analogRead(CAPTURE_BTN) <1000) {
    if(millis() - buttontimer > 1000){
      break;
      }
    }
    if(is_btn){
      is_btn=0;
      long pressDuration = millis() - buttontimer;
      if (pressDuration < 1000) {//quick relese = record
        if(recording==1){
          recording = 0;
        }
        else{
          sensor_t *s = esp_camera_sensor_get();
          s->set_framesize(s, (framesize_t)9);//VGA 800x600
          recording = 1;
        }
      } else {//long release = photo
        incrementtracker();
        takepic();
      }
    }
  }
  delay(100);
  
}

//CORE 0

//LED BLINKING RECORDING
void blinkloop(void *parameter){
  Serial.print("BLINK LOOP using core ");  Serial.print(xPortGetCoreID());
  Serial.print(", priority = "); Serial.println(uxTaskPriorityGet(NULL));
  while(true){
    if (recording==1){
      digitalWrite(MAIN_LED, HIGH);  
      delay(500);                      
      digitalWrite(MAIN_LED, LOW);   
      delay(500); 
    }
    else{
      if (sdfail){
        digitalWrite(MAIN_LED, HIGH);  
      }
      else{
        digitalWrite(MAIN_LED, LOW);
      }
      vTaskDelay(1000/portTICK_PERIOD_MS);
    }
  }
}

//AUDIO LOOP
void audioloop(void *parameter){
  Serial.print("AUDIO LOOP using core ");  Serial.print(xPortGetCoreID());
  Serial.print(", priority = "); Serial.println(uxTaskPriorityGet(NULL));
  
  while (is_mic){
    
    if(audiorecord){
      if(i2s_go){
        if (audiobuffleng + SAMPLE_BUFFER_SIZE > SAMPLE_BUFFER_SIZE*10){
          memset(bigbuff, 0, SAMPLE_BUFFER_SIZE * 10 * sizeof(int16_t));
          audiobuffleng=0;
        }

        size_t bytes_read = 0;
        i2s_read(I2S_NUM_0, audiobuff, sizeof(int16_t) * SAMPLE_BUFFER_SIZE, &bytes_read, portMAX_DELAY);
        memcpy(bigbuff + audiobuffleng, audiobuff, sizeof(int16_t) * SAMPLE_BUFFER_SIZE);
        audiobuffleng += SAMPLE_BUFFER_SIZE;
        
        
      }
      else{
        wavfile.write((const byte *)bigbuff, audiobuffleng * sizeof(int16_t));
        memset(bigbuff, 0, SAMPLE_BUFFER_SIZE * 10 * sizeof(int16_t));
        audiobuffleng=0;
        i2s_go=true;
      }

    }
    vTaskDelay(portTICK_PERIOD_MS);
  }
}

//CAMERA MAIN LOOP
void cameraloop(void *parameter) {
  Serial.print("CAMERA LOOP using core ");  Serial.print(xPortGetCoreID());
  Serial.print(", priority = "); Serial.println(uxTaskPriorityGet(NULL));
  int isstopped = 0;
  
  while (true) {
    //NOT RECORDING
    if (recording == 0 && frame_cnt ==0){
      if (isstopped ==0){
        Serial.println("NOT RECORDING");
        isstopped=1;
      }
    }//START RECORD
    else if(recording==1 && frame_cnt==0){
      incrementtracker();
      isstopped=0;
      wavfile = SD_MMC.open("/"+String(lastfilename)+".wav", "w");
      sensor_t * s = esp_camera_sensor_get();
      framesize = s->status.framesize ;
      quality = s->status.quality ;
      Serial.print("avi resolution = ");
      Serial.print(framesize);
      Serial.print(" avi quality = ");
      Serial.println(quality);

      frame_cnt++;      

      fb_curr = get_good_fb();

      aviStart();
      audiorecord = 1;

      fb_next = get_good_fb(); 

      sd_go=1;
      
    }//SAVE ANOTHER FRAME
    else if(recording == 1 && frame_cnt>0){
      long fpscounter = millis();
      if(camera_go==1){
        camera_go=0;
        frame_cnt++;

        esp_camera_fb_return(fb_curr);

        fb_curr = fb_next;

        sd_go=1;
        fb_next = get_good_fb();  
      }
      if (millis()-fpscounter==0){
        fpscounter = 1;
      }
      fps = 1000/(millis()-fpscounter);
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
        audiorecord = 0;
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
      another_save_avi(fb_curr);    // do the actual sd wrte
      i2s_go = false;               // saving i2s buffer
      camera_go=1;
    }
    vTaskDelay(portTICK_PERIOD_MS);
  }
}


////////////////////////////////////////////////// MAIN LOOP END  ///////////////////////////////////////////// 


////////////////////////////////////////////// SETUP / INIT /////////////////////////////////////
//initialize SD for the first time / setup
void SD_init(){
  SD_MMC.setPins(SDMMC_CLK, SDMMC_CMD, SDMMC_D0 , SDMMC_D1, SDMMC_D2, SDMMC_D3);
  //if u use only 1 bit sdio mode
  /////////////////"/sdcard", true , false, 20000, 5
  if(!SD_MMC.begin("/sdcard", false, false, 40000, 5)){//its default config
    Serial.println("Card Mount Failed");
    sdfail = 1;
    return;
  }
  else{//make sure you have already have tracker.txt on sdcard, with number in it
    File file = SD_MMC.open("/tracker.txt");
    
    if (file) {
      if (file.available()) {
        lastfilename = file.parseInt();
      }
        
      else {
        Serial.println("Gagal membuka file untuk dibaca.");
      }
      Serial.print("Nomor file terakhir ");
      Serial.println(lastfilename);
      file.close();
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
void wifisetup(){
    File file = SD_MMC.open("/wifi.txt");
    String ssid = file.readStringUntil('\n');
    ssid.trim(); // remove whitespace
    String password = file.readStringUntil('\n');
    password.trim();
    file.close();
}

//ALL OTHER SETUP
void initial_setup(){
  SD_init();
  wifisetup();
  //BTN manual start
  pinMode(CAPTURE_BTN, INPUT);
  pinMode(MAIN_LED, OUTPUT);
  delay(500);//charging capacitor for low power supply!
  digitalWrite(MAIN_LED, HIGH);
}
void custom_setup(){

  if(is_mic){
    i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_NUM_0, &i2s_mic_pins);
  }
  Serial.print("setup using core ");  Serial.print(xPortGetCoreID());
  Serial.print(", priority = "); Serial.println(uxTaskPriorityGet(NULL));
  //website that work on port 8000
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    String html = "<!DOCTYPE html><html lang=\"en\">";
    html += "<head><meta charset=\"UTF-8\"><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">";
    html += "<title>Nomor</title></head>";
    html += "<body><h1>Nomor file: " + String(lastfilename) + "</h1>";
    html += "<p>Status: Not Recording </p>";
    html += "<form action=\"/start\" method=\"get\"><button type=\"submit\">Start Record</button></form>";

    html += "<form action=\"/file\" method=\"get\"><button type=\"submit\">Download File</button></form>";
    html += "</body></html>";
    recording = 0;

    request->send(200, "text/html", html);
  });

  server.on("/start", HTTP_GET, [](AsyncWebServerRequest* request) {
    recording = 1;
    
    String html = "<!DOCTYPE HTML><html><head><title>ESP32 Web Server - Rekam</title>";
    html += "<meta http-equiv=\"refresh\" content=\"1\"></head><body>";
    html += "<h1>Rekaman Dimulai</h1>";
    html += "<p>Lama Rekam:"+ String ((millis()-startms)/1000) +" detik </p>";
    html += "<p>FPS:"+ String (fps) +" </p>";
    html += "<button onclick=\"location.href='/'\">Stop Recording</button>";
    html += "</body></html>";
    request->send(200, "text/html", html);
  });



  server.on("/file", HTTP_GET, [](AsyncWebServerRequest *request){
    String html = "<html><body><h1>SD Card Files</h1><p>Change filename + extension after download done!</p><ul>";
    File root = SD_MMC.open("/");
    File file = root.openNextFile();
    while (file) {
      if (!file.isDirectory()) {
        html += "<li><a href=\"/download?file=" + String(file.name()) + "\">" + String(file.name()) + "</a></li>";
      }
      file = root.openNextFile();
    }
    html += "</ul></body></html>";
    request->send(200, "text/html", html);
  });

  server.on("/download", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("file")) {
      String filename = request->getParam("file")->value();
      filename = "/"+filename;
      if (SD_MMC.exists(filename)) {
        request->send(SD_MMC, filename, "application/octet-stream");
      } else {
        request->send(404, "text/plain", "File not found");
      }
    } else {
      request->send(400, "text/plain", "Bad Request");
    }
  });
  
  
  server.begin();//server on port 8000

  //RUNNING ON CORE 0
  xTaskCreatePinnedToCore(
    cameraloop,     // Fungsi untuk task1
    "Cameraloop",   // Nama task1
    10000,     // Ukuran stack
    NULL,      // Parameter task1
    3,         // Prioritas task1
    &Cameraloop,    // Handle task1
    0          // Jalankan pada core 0
  );
  delay(100);
  xTaskCreatePinnedToCore(
    audioloop,     // Fungsi untuk task1
    "Audioloop",   // Nama task1
    10000,     // Ukuran stack
    NULL,      // Parameter task1
    4,         // Prioritas task1
    &Audioloop,    // Handle task1
    1          // Jalankan pada core 0
  );
  delay(100);
  xTaskCreatePinnedToCore(
    blinkloop,     // Fungsi untuk task1
    "Blinkloop",   // Nama task1
    10000,     // Ukuran stack
    NULL,      // Parameter task1
    2,         // Prioritas task1
    &Blinkloop,    // Handle task1
    1          // Jalankan pada core 0
  );
  delay(100);

  // RUNNING ON CORE 1
  xTaskCreatePinnedToCore(
    sdloop,     // Fungsi untuk task2
    "SDloop",   // Nama task2
    10000,     // Ukuran stack
    NULL,      // Parameter task2
    3,         // Prioritas task2
    &SDloop,    // Handle task2
    1          // Jalankan pada core 1
  );
  delay(100);
  Serial.print("DEFAULT CAMERA RESOLUTION: ");
  Serial.println(framesize);
  Serial.print("DEFAULT CAMERA QUALITY: ");
  Serial.println(quality);
  digitalWrite(MAIN_LED, LOW);//stop warning light

}

////////////////////////////////////////////// END OF SETUP  /////////////////////////////////////

/////////////////////////////////////BELOW IS COPY STRAIGHT FROM ESP32 CAMERA WEB SERVER EXAMPLE//////////////////////////
#include "esp_camera.h"
#include <WiFi.h>

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

//already stated above!

void startCameraServer();
void setupLedFlash(int pin);

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();
  initial_setup();
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
  s->set_hmirror(s, 1);
#endif

  WiFi.softAP(APssid);
  WiFi.setSleep(false);

  Serial.println("");
  Serial.println("WiFi connected");

  startCameraServer();

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.softAPIP());
  Serial.println("' to connect");
  
  //this line is not part of camerawebserver example!
  framesize = s->status.framesize ;
  quality = s->status.quality ;
  custom_setup();
}

void loop() {
  buttonloop();
}
