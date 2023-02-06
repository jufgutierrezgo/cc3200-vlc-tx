

/*
    WiFi AP-mode Connect/Disconnect monitor

  This sketch will put the CC3200 or CC3100 in Access Point mode and report
  when WiFi clients (cellphone, laptop, etc) connect to the Access Point and when
  they disconnect.

  Connections trigger a serial dump of every connected client with their IP and MAC.
  Disconnections only trigger a simple notification, as the "last disconnected client"
  information is not saved anywhere.

  Created 11/11/2014 by Eric Brundick for the Energia project.
*/

#ifndef __CC3200R1M1RGC__
// No need to include SPI.h for CC3200
#endif
#include <WiFi.h>
#include <SPI.h>

//Library for timer enable
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_timer.h"
#include "inc/hw_types.h"
#include "driverlib/prcm.h"
#include "driverlib/debug.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"

#define NO_FRAMES 2000
#define L_FRAME 240

#define stack16 "0123456789abcdef"
#define stack8 "01234567"
#define stack4 "0689"

void setTimer(void (*timer_func)(void), uint32_t freq);
void startTimer(void);
void stopTimer(void);
void sync(void);
//void vlc_csk(void);




/*** VARIABLES FOR WEB SERVER AND AP WIFI ***/

const char ssid[] = "AP2VLC";
const char wifipw[] = "supermario";



boolean flag_luminaire = LOW;
boolean flag_client_first = HIGH;

char buffer_temp [150] = {
  0
};

WiFiServer server(80);

/*** VARIABLE FOR MODULATION CONTROL ***/
/**** Data for Tx ******/
//const String VLC_Message_CSK = String("---VLC@jufgutierrezgo***electronic-engineer***master-in-science-of-telecomunication***matisse-laboratory***universidad-nacional de colombia***the-test-frame-to-demostrate-the-optical-link-with-color-shift-keying***enjoy-this-new-tech++++");
//const String VLC_Message_CSK = String("*******VLC@jufgutierrezgo***electronic-engineer***master-in-science-of-telecomunication***");
const String VLC_Message_CSK = String("*******VLC@jufgutierrezgo******master-in-science-of-telecomunication*******");
//const String VLC_Message_CSK = String("*809809fedcba9876543210fedcba9876543210fedcba9876543210fedcba9876543210@@@@");
//const  String VLC_Message_OOK = String("+++VLC****@jufgutierrezgo*****optical-link-on-off-keying++++");
//const  String VLC_Message_OOK = String("VLC@jufgutierrezgo**mod-ook***");
const  String VLC_Message_OOK = String("This-is-a-new-message-jfg2****");
//char buffer_csk[] = {"2a2a2a2a2a2a2a564c43406a756667757469657272657a676f2a2a2a2a2a2a6d61737465722d696e2d736369656e63652d6f662d74656c65636f6d756e69636174696f6e2a2a2a2a2a2a2a"};
//char buffer_16csk[] = {"XX8090123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef0123456789a"};
//This array is used when the receiver only read 80 symbols (240 adc-data values)
char buffer_16csk[] = {"XX8090123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef0123456789a"};
char buffer_8csk[] =  {"XX745012345670123456701234567012345670123456701234567012345670123456701234567012"};
char buffer_4csk[] =  {"XX809680968096809680968096809680968096809680968096809680968096809680968096809680"};
//char buffer_16csk[] = {"XX80900000000000000000000111111111111222222222222233333333334444444444555555556666666777777777888888889999999aaaaaaaabbbbbbbbbbccccccccccccdddddddddddddddeeeeee"};
//char buffer_16csk[] = {"9999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999"};
//char buffer_16csk[] = {"XXXXXXXXXXXXXXXXXXXXXXXXXXXXXX8888888888888888888800000000000000000000000000000000000000000000000000000000999999999999999999999999999999999999999999999999999999"};
//char buffer_4csk[] = {"XX8800990609966696096060099800690008886868080689099666688998698699888066090000669609960668688006668069098680698806886690999066808086989968008869889980"};
char buffer_ook[L_FRAME / 2] = {
  0
};
char buffer_man[2 * L_FRAME] = {
  0
};

//const char info[] = "This is the VLC message for test the wireless communication using visible spectrum: HELLO WORLD :)";
uint8_t mod = 0; // 0-> NOT INIZIALIZED 1->OOK 2->CSK
// value for the frequency for transmitting the VLC
uint32_t freq_vlc = 1000;
boolean flag_frequency = LOW;
boolean flag_start_vlc = LOW;
boolean flag_16csk = LOW;
boolean flag_8csk = LOW;

unsigned int wlevel_ook = 2000;
uint8_t sym = 16;
//This variable is the flag control for select the function timer for the VLC tranmission
//0 -> Not Initialized
//1 -> OOK Tx
//2 -> CSK Tx
//3 -> test OOK
//4 -> test CSK
uint8_t select_timer_func = 0;

//This variable counts the number of frame that have been sent-limit to NO_FRAMES
uint8_t count_frame = 0;

/*
  //Using currently linear regression WHITE 0
  //EDITED MANUALLY
  const uint16_t Pijk[16][3] = {
  600,  455,  170,
  155,  645,  55,
  0,  657,  120,
  400,  590,  0,
  200,  558,  190,
  0,  711,  0,
  635,  480,  75,
  1130, 240,  315,
  0,  535,  360,
  360,  295,  700,
  0,  0,  1829,
  1310, 0,  705,
  1010, 385,  0,
  1600, 210,  115,
  2070, 0,  292,
  2800, 0,  0
  };*/


//Using currently linear regression WHITE 0 L=6cd/m2
//Computed from power matrix for 16-CSK Constellations
const uint16_t Pijk[16][3] = {
  847, 237, 610,
  282, 553, 203,
  0, 474, 610,
  847, 474,   0,
  282, 316, 813,
  0, 711,   0,
  1129, 316, 203,
  1129,  79, 813,
  0, 237, 1219,
  282,  79, 1423,
  0,   0, 1829,
  847,   0, 1219,
  1694, 237,   0,
  1976,  79, 203,
  1694,   0, 610,
  2541,   0,   0
};
/* */

/*
  //Using currently linear regression WHITE 0 L=3cd/m2
  //Computed from power matrix for 16-CSK Constellations
  const uint16_t Pijk[16][3] = {
  424, 118, 305,
  141, 276, 102,
   0, 237, 305,
  424, 237,   0,
  141, 158, 406,
   0, 355,   0,
  565, 158, 102,
  565,  39, 406,
   0, 118, 610,
  141,  39, 711,
   0,   0, 915,
  424,   0, 610,
  847, 118,   0,
  988,  39, 102,
  847,   0, 305,
  1271,   0,   0
  };
  /* */

/*
  //Using currently linear regression WHITE 0 L=2cd/m2
  //Computed from power matrix for 16-CSK Constellations
  const uint16_t Pijk[16][3] = {
  282,  79, 203,
  94, 184,  68,
   0, 158, 203,
  282, 158,   0,
  94, 105, 271,
   0, 237,   0,
  376, 105,  68,
  376,  26, 271,
   0,  79, 406,
  94,  26, 474,
   0,   0, 610,
  282,   0, 406,
  565,  79,   0,
  659,  26,  68,
  565,   0, 203,
  847,   0,   0
  };

*/


//Using currently linear regression WHITE 0 L=6cd/m2
//Computed from power matrix for 16-CSK Constellations
const uint16_t Pijk8[16][3] = {
  0, 711,   0,
  0, 474, 610,
  847, 474,   0,
  1553, 197, 203,
  0,   0, 1829,
  282, 197, 1118,
  1271,   0, 915,
  2541,   0,   0
};
/* */

/*
  //Using currently linear regression WHITE 0 L=3cd/m2
  //Computed from power matrix for 16-CSK Constellations
  const uint16_t Pijk8[16][3] = {
   0, 355,   0,
   0, 237, 305,
  424, 237,   0,
  776,  99, 102,
   0,   0, 915,
  141,  99, 559,
  635,   0, 457,
  1271,   0,   0
  };
  /* */

/*
  //Using currently linear regression WHITE 0 L=2cd/m2
  //Computed from power matrix for 16-CSK Constellations
  const uint16_t Pijk8[16][3] = {
   0, 237,   0,
   0, 158, 203,
  282, 158,   0,
  518,  66,  68,
   0,   0, 610,
  94,  66, 373,
  424,   0, 305,
  847,   0,   0
  };
  /* */

//Linear regression for WHITE 1000 L=11cd/m² for RGB channel
//EDITED MANUALLY
const uint16_t Pijk_w1000[16][3] = {
  790,  310,  330,
  210,  510,  130,
  0,  500,  290,
  630,  450,  0,
  340,  380,  420,
  0,  643,  0,
  940,  370,  170,
  1240, 220,  520,
  0,  350,  730,
  410,  210,  1040,
  0,  0,  1674,
  1170, 0,  912,
  1370, 300,  0,
  2024, 210,  210,
  1850, 0,  386,
  2571, 0,  0
};


/*** VARIABLES FOR DAC CONTROL ***/


//static unsigned int* rgbw_val;
unsigned int rgbw_val[4] = {
  0
};

// set pin 8 as the slave select for the digital pot:
const int SYNC_PIN = 2;

//Variables for WTM and WRM modes DAC
//ch_i -> channel E DAC
//ch_j -> channel F DAC
//ch_k -> channel G DAC
//ch_w -> channel H DAC
char WTM_LSB = 0x00;
char WTM_MSB = 0x90;
char WRM_LSB = 0x00;
char WRM_MSB = 0x80;

char SIM_MSB = 0xA0;
char SIM_LSB = 0xF0;

char ch_i = 0x04;
char ch_j = 0x05;
char ch_k = 0x06;
char ch_w = 0x07;
//  int i = 0;

//Variables for voltage for each symbol
float vi = 0;
float vj = 0;
float vk = 0;
float vw = 0;
/*
  //Corresponding value in decimal and hexa representation for ouput voltage DAC
  int16_t decimal_i;
  int16_t decimal_j;
  int16_t decimal_k;
  int16_t decimal_w;*/

uint8_t hexa_i[3];
uint8_t hexa_j[3];
uint8_t hexa_k[3];
uint8_t hexa_w[3];

char csk_type = '4';

//Voltage in the DAC output for each center bands and Y=0.8
// The current in LED correspond to V_X/2
//float V_I = (1.1)*0.41;
//float V_J = (1.1)*0.3;
//float V_K = (1.1)*1.22;
//float V_W = (1.1)*1.22;


void setup()
{

  pinMode(PUSH2, INPUT_PULLUP);
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);

  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(YELLOW_LED, LOW);


  setGPIO();
  init_DAC();



  /************* Serial Wifi Printer ******************/
  Serial.begin(115200);
  Serial.print("Setting up Access Point named: ");
  Serial.println(ssid);
  Serial.print("AP uses WPA and password is: ");
  Serial.println(wifipw);

  WiFi.beginNetwork((char *)ssid, (char *)wifipw);

  while (WiFi.localIP() == INADDR_NONE) {
    // print dots while we wait for the AP config to complete
    Serial.print('.');
    delay(300);
  }

  Serial.println();
  Serial.println("AP active.");

  printWifiStatus();

  Serial.println("Starting webserver on port 80");
  server.begin();                           // start the web server on port 80
  Serial.println("Webserver started!");
  /*****************************************************/



  //*****************************************************/
  // Convert the character data to hexa
  //
  /*
    for (int i =0; i<VLC_Message_CSK.length();i++){

    String aux2 = String(VLC_Message_CSK[i],HEX);

    for(int j=0;j<aux2.length();j++) buffer_csk[2*i+j] = aux2[j];
    Serial.println(aux2);


    }

    Serial.println(buffer_csk);

  */

  //*****************************************************/
  // Convert the character data to bin
  //



  for (int i = 0; i < sizeof(buffer_ook); i++) buffer_ook[i] = '0';

  for (int i = 0; i < VLC_Message_OOK.length(); i++) {

    String aux2 = String(VLC_Message_OOK[i], BIN);
    // Serial.println(aux2);
    for (int j = aux2.length(); j > 0; j--)
      buffer_ook[8 * i + (8 - j)] = aux2[aux2.length() - j];

    //Serial.println(VLC_Message_CSK.length());
  }

  /*******************************************************/

  manchester();

  /*
    // Print biffers
    Serial.println("Buffer 4CSK");
    Serial.println(buffer_4csk);
    Serial.println("Buffer OOK");
    Serial.println(buffer_ook);
    Serial.println("Buffer Manchester");
    Serial.println(buffer_man);
  */
}

unsigned int num_clients = 2;

void loop()
{



  unsigned int a, i;

  // ********************************************************************

  int j = 0;
  WiFiClient client = server.available();   // listen for incoming clients

  if (client) {                             // if you get a client,
    Serial.println("new client");           // print a message out the serial port
    char buffer[150] = {
      0
    };                 // make a buffer to hold incoming data
    while (client.connected()) {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);          // print it out the serial monitor
        //Serial.write('%');
        if (c == '\n') {                    // if the byte is a newline character
          //Serial.write('%');
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (strlen(buffer) == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:

            //if(flag_client_first){

            client.println("HTTP/1.1   200 OK");
            client.println("Content-type:text/html");
            client.println();

            // the content of the HTTP response follows the header:
            client.println("<html><head><title>VLC CC3200 WiFi Web Server</title></head><body align=center>");
            client.println("<h1 align=center><font color=\"red\">Welcome to the WiFi VLC Web Server</font></h1>");
            client.println("<h2 align=left><font color=\"black\">In this page, you can download an image using visible light communication download link.</font></h2>");
            client.println("<h2 align=left><font color=\"black\">Also, you can control the Luminous Flux RGBW Luminaire and test the Wifi connection blinking RED LED on CC3200 Lauchpad.</font></h2>");

            client.print("<h3 align=left><font color=\"blue\">*** CONTROL RGBW VALUES FOR LUMINAIRE (DECIMAL VALUES 0-1400)*** <br> </font></h3>");

            client.println("<iframe name='votar' style='display:none;'></iframe>");
            client.println("<form action='/set_rgbw_page.php' method='HEAD' target='votar'>");
            client.print("RED CHANNEL  <br>");
            client.print("<input type='number' name='rval' min='0' max='4096' value='0'> <br>");
            client.println("GREEN CHANNEL  <br>");
            client.print("<input type='number' name='gval' min='0' max='4096' value='0'> <br>");
            client.print("BLUE CHANNEL  <br>");
            client.print("<input type='number' name='bval' min='0' max='4096' value='0'> <br>");
            client.print("WHITE CHANNEL  <br>");
            client.print("<input type='number' name='wval' min='0' max='4096' value='0'> <br>");
            client.print("<button onclick='myFunction()' >SET</button><br><br>");
            client.println("</form>");

            client.print("<h3 align=left><font color=\"blue\">*** CONTROL RGBW VALUES FOR 16-CSK SYMBOLS*** <br> </font></h3>");
            client.print("SYMBOLS FOR 16 CSK  <br>");
            client.println("<iframe name='csk-symbol-16' style='display:none;'></iframe>");
            client.println("<form action='/csk_symbol_16.php' method='HEAD' target='csk-symbol-8'>");
            client.print(" <select name='symbol'>");
            client.print("<option value='06'>S0-0110</option>");
            client.print("<option value='01'>S1-0001</option>");
            client.print("<option value='03'>S2-0011</option>");
            client.print("<option value='05'>S3-0101</option>");
            client.print("<option value='02'>S4-0010</option>");
            client.print("<option value='00'>S5-0000</option>");
            client.print("<option value='07'>S6-0111</option>");
            client.print("<option value='14'>S7-1110</option>");
            client.print("<option value='10'>S8-1010</option>");
            client.print("<option value='11'>S9-1011</option>");
            client.print("<option value='09'>S10-1001</option>");
            client.print("<option value='15'>S11-1111</option>");
            client.print("<option value='04'>S12-0100</option>");
            client.print("<option value='13'>S13-1101</option>");
            client.print("<option value='12'>S14-1100</option>");
            client.print("<option value='08'>S15-1000</option>");
            client.print("</select><br><br>");
            client.print("<button onclick=''>SET</button><br>");
            client.println("</form>");

            client.print("SYMBOLS FOR 8-CSK <br>");

            client.println("<iframe name='csk-symbol-8' style='display:none;'></iframe>");
            client.println("<form action='/csk_symbol_8.php' method='HEAD' target='csk-symbol-8'>");
            client.print(" <select name='symbol'>");
            client.print("<option value='04'>S0-100</option>");
            client.print("<option value='00'>S1-000</option>");
            client.print("<option value='06'>S2-110</option>");
            client.print("<option value='02'>S3-010</option>");
            client.print("<option value='05'>S4-101</option>");
            client.print("<option value='01'>S5-001</option>");
            client.print("<option value='03'>S6-011</option>");
            client.print("<option value='07'>S7-111</option>");
            client.print("</select><br><br>");
            client.print("<button onclick=''>SET</button><br>");
            client.println("</form>");



            client.print("<h3 align=left><font color=\"blue\">*** RECEIVE VLC MESSAGE ***<br> </font></h2>");

            client.print("Set frequency (Default 1000Hz) <br>");
            client.println("<iframe name='freq' style='display:none;'></iframe>");
            client.println("<form action='/set_freq.php' method='HEAD' target='freq'>");
            client.print("<input type='number' name='fval' min='1' max='1000000' value='1000'> <br><br>");
            client.print("<button onclick='' >Set Freq</button><br>");
            client.println("</form>");

            client.print("Choose the modulation <br>");
            client.println("<iframe name='modulation' style='display:none;'></iframe>");
            client.println("<form action='/set_modulation.php' method='HEAD' target='modulation'>");
            client.print(" <select name='mod'>");
            client.print("<option value='OOK'>OOK</option>");
            client.print("<option value='4CSK'>4CSK</option>");
            client.print("<option value='8CSK'>8CSK</option>");
            client.print("<option value='16CSK'>16CSK</option>");
            client.print("<option value='CSK-White1000'>CSK-White1000</option>");
            client.print("<option value='testOOK'>testOOK</option>");
            client.print("<option value='test16CSK'>test16CSK</option>");
            client.print("<option value='test8CSK'>test8CSK</option>");
            client.print("<option value='test4CSK'>test4CSK</option>");
            client.print("</select><br><br>");
            client.print("<button onclick=''>DOWNLOAD</button><br>");
            client.println("</form>");

            client.println("<iframe name='stop_mod' style='display:none;'></iframe>");
            client.println("<form action='/stop_mod.php' method='HEAD' target='stop_mod'>");
            client.print("<button onclick=''>STOP</button><br>");
            client.println("</form>");


            client.println("<p id='demo'></p>");
            client.println("<script>");
            client.println("function myFunction() {");
            //client.println("document.getElementById('demo').innerHTML = 43;");
            client.println("}");
            client.println("</script>");



            //flag_client_first = LOW;

            // The HTTP response ends with another blank line:
            client.println();
            // break out of the while loop:
            break;
            //}
          }
          else {
            if ((buffer[0] == 'G') && (buffer[1] == 'E') && (buffer[2] == 'T')) buff2temp(buffer);
            // if you got a newline, then clear the buffer:
            memset(buffer, 0, 150);
            j = 0;
          }
        }
        else if (c != '\r') {    // if you got anything else but a carriage return character,
          buffer[j++] = c;      // add it to the end of the currentLine
        }

        // Check to see if the client request was "GET /H" or "GET /L":
        if (endsWith(buffer, "GET /H")) {
          digitalWrite(GREEN_LED, HIGH);               // GET /H turns the LED on
        }
        if (endsWith(buffer, "GET /L")) {
          digitalWrite(GREEN_LED, LOW);                // GET /L turns the LED off
        }

        if (endsWith(buffer, "GET /set_rgbw_page.php")) { // GET /set_rgbw_page.php for set the RGBW channels inn the luminaire
          flag_luminaire = HIGH;
        }

        if (endsWith(buffer, "GET /stop_mod.php")) { // GET for transmit using OOK modulationM
          stopTimer();
        }

        if (endsWith(buffer, "GET /set_freq.php")) { // GET /set_rgbw_page.php for set the RGBW channels inn the luminaire
          flag_frequency = HIGH;
        }

        if (endsWith(buffer, "GET /csk_symbol_16.php")) { // GET /set_rgbw_page.php for set the RGBW channels inn the luminaire
          flag_16csk = HIGH;
        }

        if (endsWith(buffer, "GET /csk_symbol_8.php")) { // GET /set_rgbw_page.php for set the RGBW channels inn the luminaire
          flag_8csk = HIGH;
        }

        if (endsWith(buffer, "GET /set_modulation.php?mod=OOK")) { // GET for transmit using OOK modulationM

          setTimer(vlc_ook, 2 * freq_vlc);
          flag_start_vlc = HIGH;
        }

        if (endsWith(buffer, "GET /set_modulation.php?mod=4CSK")) { // GET for transmit using OOK modulationM
          //Serial.println(buffer_csk);
          csk_type = '2';
          setTimer(vlc_csk, freq_vlc);
          flag_start_vlc = HIGH;
        }

        if (endsWith(buffer, "GET /set_modulation.php?mod=8CSK")) { // GET for transmit using OOK modulationM
          //Serial.println(buffer_csk);
          csk_type = '3';
          setTimer(vlc_csk, freq_vlc);
          flag_start_vlc = HIGH;
        }

        if (endsWith(buffer, "GET /set_modulation.php?mod=16CSK")) { // GET for transmit using OOK modulationM
          //Serial.println(buffer_csk);
          csk_type = '4';
          setTimer(vlc_csk, freq_vlc);
          flag_start_vlc = HIGH;
        }

        if (endsWith(buffer, "GET /set_modulation.php?mod=CSK-White1000")) { // GET for transmit using OOK modulationM
          setTimer(vlc_csk_white1000, freq_vlc);
          flag_start_vlc = HIGH;
        }

        if (endsWith(buffer, "GET /set_modulation.php?mod=testOOK")) { // GET for transmit using OOK modulationM
          //freq_vlc = 2*freq_vlc;
          setTimer(test_ook, 2 * freq_vlc);
          //setTimer();
          flag_start_vlc = HIGH;
        }

        if (endsWith(buffer, "GET /set_modulation.php?mod=test16CSK")) { // GET for transmit using OOK modulationM
          csk_type = '4';
          setTimer(test_csk, freq_vlc);
          flag_start_vlc = HIGH;
        }
        if (endsWith(buffer, "GET /set_modulation.php?mod=test8CSK")) { // GET for transmit using OOK modulationM
          csk_type = '3';
          setTimer(test_csk, freq_vlc);
          flag_start_vlc = HIGH;
        }
        if (endsWith(buffer, "GET /set_modulation.php?mod=test4CSK")) { // GET for transmit using OOK modulationM
          csk_type = '2';
          setTimer(test_csk, freq_vlc);
          flag_start_vlc = HIGH;
        }
        if (endsWith(buffer, "GET /set_modulation.php?mod=testCSK-White1000")) { // GET for transmit using OOK modulationM
          setTimer(test_csk_white1000, freq_vlc);
          flag_start_vlc = HIGH;
        }

      }
    }

    //Serial.println(buffer_temp);
    //Serial.println("XYZ");

    // Set the RGBW values
    if (flag_luminaire) {
      get_valRGBW(buffer_temp);
      decimal2hexa(rgbw_val[0], rgbw_val[1], rgbw_val[2], rgbw_val[3]);
      setAPI_DAC('A');
      flag_luminaire = LOW;
    }
    // Set frequency of modulation
    if (flag_frequency) {
      get_freq(buffer_temp);
      flag_frequency = LOW;
    }
    // Start to transmitt
    if (flag_start_vlc) {
      flag_start_vlc = LOW;
      startTimer();
    }
    // Set a DC symbols of 16-CSK
    if (flag_16csk) {
      get_sym16csk(buffer_temp);
      xy_16mapping(stack16[sym]);
      flag_16csk = LOW;
    }
    // Set a DC symbols of 8-CSK
    if (flag_8csk) {
      get_sym16csk(buffer_temp);
      xy_8mapping(stack8[sym]);
      flag_8csk = LOW;

    }

    //Serial.println(rgbw_val);
    // close the connection:
    client.stop();
    //Serial.println("client disonnected");
  }


  // ********************************************************************
  a = WiFi.getTotalDevices();

  // Did a client connect/disconnect since the last time we checked?
  if (a != num_clients) {
    if (a > num_clients) {  // Client connect
      digitalWrite(GREEN_LED, !digitalRead(GREEN_LED));
      Serial.println("Client connected! All clients:");
      for (i = 0; i < a; i++) {
        Serial.print("Client #");
        Serial.print(i);
        Serial.print(" at IP address = ");
        Serial.print(WiFi.deviceIpAddress(i));
        Serial.print(", MAC = ");
        Serial.println(WiFi.deviceMacAddress(i));
        //webpage();

      }
    }
    else {  // Client disconnect
      digitalWrite(GREEN_LED, !digitalRead(GREEN_LED));
      //flag_client_first = LOW;
      Serial.println("Client disconnected.");
    }
    num_clients = a;
  }

  delay(100);

  // **************************************************************

}


//
//a way to check if one array ends with another array
//
boolean endsWith(char* inString, char* compString) {
  int compLength = strlen(compString);
  int strLength = strlen(inString);

  //compare the last "compLength" values of the inString
  int i;
  for (i = 0; i < compLength; i++) {
    char a = inString[(strLength - 1) - i];
    char b = compString[(compLength - 1) - i];
    if (a != b) {
      return false;
    }
  }
  return true;
}

void get_valRGBW(char* inString) {

  char temp_val[5] = {
    0
  };
  unsigned int rgbw_val_temp[4];

  /*
    temp_val[0] = 0;
    temp_val[1] = 0;
    temp_val[2] = 0;
    temp_val[3] = 0;
  */

  //Serial.println(inString);

  int strLength = strlen(inString);
  int h = 0;
  int j = 0;
  int k = 0;
  int count_read = 0;

  while (h < strLength) {
    temp_val[0] = 0;
    temp_val[1] = 0;
    temp_val[2] = 0;
    temp_val[3] = 0;
    if (inString[h] == '=')j = h + 1;
    if (inString[h] == '&')
    {
      for (k = (h - 1); k >= j; k--) temp_val[k - j] = inString[k];
      //Serial.println(strlen(temp_val));
      /*Serial.println(temp_val[0]);
        Serial.println(temp_val[1]);
        Serial.println(temp_val[2]);
        Serial.println(temp_val[3]);*/
      //Serial.println('a');
      rgbw_val[count_read] =  str2int(temp_val);
      count_read++;
    }
    if (inString[h] == 'H')
    {
      for (k = (h - 2); k >= j; k--) temp_val[k - j] = inString[k];
      //Serial.println(strlen(temp_val));
      /*Serial.println(temp_val[0]);
        Serial.println(temp_val[1]);
        Serial.println(temp_val[2]);
        Serial.println(temp_val[3]);*/
      rgbw_val[count_read] =  str2int(temp_val);
      //count_read++;

    }
    h++;
  }

  //Serial.print("NANIA :) salsa");

  //Serial.println(temp_val);
  Serial.print("Decimal value of RGBW setter:\n");
  Serial.println(rgbw_val[0]);
  Serial.println(rgbw_val[1]);
  Serial.println(rgbw_val[2]);
  Serial.println(rgbw_val[3]);



  memset(buffer_temp, 0, 150);
  //return rgbw_val;
}



void get_freq(char* inString) {

  char temp_val[7] = {
    0, 0, 0, 0, 0, 0, 0
  };
  //temp_val[7]={0,0,0,0,0,0,0};
  //Serial.println(inString);

  int strLength = strlen(inString);
  int h = 0;
  int j = 0;
  int k = 0;
  int count_read = 0;

  while (h < strLength) {
    if (inString[h] == '=')j = h + 1;
    if (inString[h] == 'H')
    {
      for (k = (h - 2); k >= j; k--) temp_val[k - j] = inString[k];

      freq_vlc =  str2int(temp_val);

    }
    h++;
  }

  Serial.println(freq_vlc);

  memset(buffer_temp, 0, 150);

}




void get_sym16csk(char* inString) {

  char temp_val[2] = {
    0
  };

  //Serial.println(inString);

  int strLength = strlen(inString);
  int h = 0;
  int j = 0;
  int k = 0;
  int count_read = 0;

  while (h < strLength) {
    if (inString[h] == '=')j = h + 1;
    if (inString[h] == 'H')
    {
      for (k = (h - 2); k >= j; k--) temp_val[k - j] = inString[k];

      sym =  str2int(temp_val);

    }
    h++;
  }

  Serial.println(sym);


  memset(buffer_temp, 0, 150);

}



uint32_t str2int(char* convStr) {
  uint32_t conv_val = 0;
  unsigned int convLength = strlen(convStr);
  unsigned int m;

  for (m = 0; m < convLength; m++) {
    //Serial.println(convLength);
    conv_val = ((convStr[m] - 48) * power_op(10, convLength - m - 1)) + conv_val;
  }
  return conv_val;
}

int power_op(int base, int n) {
  int count = 0;
  int result = 1;
  if (n == 0) return 1;
  else {
    for (count = 1; count <= n; count++) result = base * result;
    return result;
  }
}
void buff2temp(char* inString) {

  int strLength = strlen(inString);
  int i;

  for (i = 0; i < strLength; i++) buffer_temp[i] = inString[i];

  //Serial.print(buffer_temp);

}



void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
  // print where to go in a browser:
  Serial.print("To see this page in action, open a browser to http://");
  Serial.println(ip);
}

void setGPIO() {
  pinMode(RED_LED, OUTPUT);  // LED will toggle when clients connect/disconnect
  digitalWrite(RED_LED, LOW);

  pinMode(YELLOW_LED, OUTPUT);  // LED will toggle when clients connect/disconnect
  digitalWrite(YELLOW_LED, LOW);

  pinMode(GREEN_LED, OUTPUT);  // LED will toggle when clients connect/disconnect
  digitalWrite(GREEN_LED, LOW);
}

// *****************tw*************************************
void init_DAC() {

  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV1);
  SPI.setDataMode(SPI_MODE1);
  pinMode(SYNC_PIN, OUTPUT);
  set_ModeDAC(WRM_LSB, WRM_MSB);
  //set_ModeDAC(WTM_LSB,WTM_MSB);

}

// **************************************************************
void set_ModeDAC(int mode_lsb, int mode_msb) {

  // take the SS pin low to select the chip:
  digitalWrite(SYNC_PIN, LOW);

  //  send in the address and value via SPI:
  //SPI.transfer(WTM);
  SPI.transfer(mode_msb);
  SPI.transfer(mode_lsb);

  // take the SS pin high to de-select the chip:
  digitalWrite(SYNC_PIN, HIGH);
}

// **************************************************************
void set_OutputDAC(uint8_t channel_byte4, uint8_t channel_byte3, uint8_t channel_byte2, uint8_t channel_byte1) {

  // take the SS pin low to select the chip:
  digitalWrite(SYNC_PIN, LOW);

  //  send in the address and value via SPI:
  //SPI.transfer(WTM);
  SPI.transfer((channel_byte4 << 4) | channel_byte3);
  SPI.transfer((channel_byte2 << 4) | channel_byte1);

  // take the SS pin high to de-select the chip:
  digitalWrite(SYNC_PIN, HIGH);
}

// **************************************************************
void setAPI_DAC(char sel) {
  // sel is the variable to choose the configuration of the DAC
  // sel=A-> Control the luminous flux RGBW steady state
  // sel=B-> Modulation OOK
  // sel=C-> Modulation CSK

  switch (sel) {

    case 'A':
      set_OutputDAC(ch_i, hexa_i[2], hexa_i[1], hexa_i[0]);
      set_OutputDAC(ch_j, hexa_j[2], hexa_j[1], hexa_j[0]);
      set_OutputDAC(ch_k, hexa_k[2], hexa_k[1], hexa_k[0]);
      set_OutputDAC(ch_w, hexa_w[2], hexa_w[1], hexa_w[0]);
      set_ModeDAC(SIM_LSB, SIM_MSB);
      break;

    case 'B':
      set_OutputDAC(ch_w, hexa_w[2], hexa_w[1], hexa_w[0]);
      set_ModeDAC(SIM_LSB, SIM_MSB);
      break;

    case 'C':
      set_OutputDAC(ch_i, hexa_i[2], hexa_i[1], hexa_i[0]);
      set_OutputDAC(ch_j, hexa_j[2], hexa_j[1], hexa_j[0]);
      set_OutputDAC(ch_k, hexa_k[2], hexa_k[1], hexa_k[0]);
      set_ModeDAC(SIM_LSB, SIM_MSB);
      break;
  }



}

// **************************************************************
void decimal2hexa(unsigned int decimal_i, unsigned int decimal_j, unsigned int decimal_k, unsigned int decimal_w) {
  int aux1, aux2, aux3;

  hexa_i[2] = decimal_i >> 8;
  hexa_i[1] = (decimal_i << 8) >> 12;
  hexa_i[0] = (decimal_i << 12) >> 12;

  hexa_j[2] = decimal_j >> 8;
  hexa_j[1] = (decimal_j << 8) >> 12;
  hexa_j[0] = (decimal_j << 12) >> 12;

  hexa_k[2] = decimal_k >> 8;
  hexa_k[1] = (decimal_k << 8) >> 12;
  hexa_k[0] = (decimal_k << 12) >> 12;

  hexa_w[2] = decimal_w >> 8;
  hexa_w[1] = (decimal_w << 8) >> 12;
  hexa_w[0] = (decimal_w << 12) >> 12;

  Serial.print("Hexadecimal value of RGBW setter:\n");
  Serial.println(hexa_i[2]);
  Serial.println(hexa_i[1]);
  Serial.println(hexa_i[0]);

}

void setTimer(void (*timer_func)(void), uint32_t freq) {
  //void setTimer(){
  //uint16_t hola = F_CPU/freq;
  //Serial.print(hola);

  PRCMPeripheralClkEnable(PRCM_TIMERA0, PRCM_RUN_MODE_CLK);
  PRCMPeripheralReset(PRCM_TIMERA0);

  TimerConfigure(TIMERA0_BASE, TIMER_CFG_A_PERIODIC);
  TimerPrescaleSet(TIMERA0_BASE, TIMER_A, 0);
  TimerIntRegister(TIMERA0_BASE, TIMER_A, (*timer_func));
  //TimerIntRegister(TIMERA0_BASE,TIMER_A,test_ook);
  TimerIntEnable(TIMERA0_BASE, TIMER_TIMA_TIMEOUT);
  TimerLoadSet(TIMERA0_BASE, TIMER_A, F_CPU / freq);

}

void startTimer() {
  Serial.println('StartTimer');
  TimerIntEnable(TIMERA0_BASE, TIMER_TIMA_TIMEOUT);
  TimerEnable(TIMERA0_BASE, TIMER_A);
}

void stopTimer() {
  TimerIntDisable(TIMERA0_BASE, TIMER_TIMA_TIMEOUT);
  TimerDisable(TIMERA0_BASE, TIMER_A);
}

void vlc_ook(void)
{

  static uint32_t count_ook  = 0;
  static uint8_t state_vlc_ook = 0;
  static boolean clock_state;

  TimerIntClear(TIMERA0_BASE, TimerIntStatus(TIMERA0_BASE, true));

  switch (state_vlc_ook) {

    case 0:
      count_ook = 0;
      clock_state = LOW;
      digitalWrite(RED_LED, clock_state);

      set_OutputDAC(ch_w, 0, 0, 0);
      set_OutputDAC(ch_i, 0, 0, 0);
      set_OutputDAC(ch_j, 0, 0, 0);
      set_OutputDAC(ch_k, 0, 0, 0);
      set_ModeDAC(SIM_LSB, SIM_MSB);
      state_vlc_ook = 1;

      break;

    case 1:
      if (count_ook == 4 * freq_vlc) {
        count_ook = 0;
        state_vlc_ook = 2;
      }
      else count_ook++;
      break;

    case 2:
      //stopTimer();

      if (count_ook == L_FRAME) {
        count_frame++;
        count_ook = 0;
        state_vlc_ook = 3;
      }
      else {
        ook_mapping(buffer_man[count_ook]);
        delayMicroseconds(5000);
        count_ook++;
        clock_state = !clock_state;
        digitalWrite(RED_LED, clock_state);
      }
      break;

    case 3:
      if (count_frame == NO_FRAMES) {
        count_frame = 0;
        stopTimer();
        state_vlc_ook = 0;
      }
      else state_vlc_ook = 0;
      break;


    default:
      break;

  }



  //Serial.println(count_csk);



}

//This function implement the transmission using CSK with buffer_4csk or buffer_16csk.
//The process is desgined using 4 states.
void vlc_csk(void)
{
  static uint32_t count_csk = 0;
  static uint8_t state_vlc_csk = 0;
  static boolean clock_state;

  TimerIntClear(TIMERA0_BASE, TimerIntStatus(TIMERA0_BASE, true));

  switch (state_vlc_csk) {

    case 0:
      count_csk = 0;
      clock_state = LOW;
      digitalWrite(RED_LED, clock_state);

      set_OutputDAC(ch_w, 0, 0, 0);
      set_OutputDAC(ch_i, 0, 0, 0);
      set_OutputDAC(ch_j, 0, 0, 0);
      set_OutputDAC(ch_k, 0, 0, 0);
      set_ModeDAC(SIM_LSB, SIM_MSB);
      state_vlc_csk = 1;

      break;

    case 1:
      if (count_csk == 1 * freq_vlc) {
        count_csk = 0;
        state_vlc_csk = 2;
      }
      else count_csk++;
      break;

    case 2:
      //stopTimer();

      if (count_csk == L_FRAME / 3 + 1) {
        count_frame++;
        count_csk = 0;
        state_vlc_csk = 3;
      }
      else {
        //xy_16mapping is used for 4mapping.
        if (csk_type == '2') xy_16mapping(buffer_4csk[count_csk]);
        if (csk_type == '3') xy_8mapping(buffer_8csk[count_csk]);
        if (csk_type == '4') xy_16mapping(buffer_16csk[count_csk]);
        delayMicroseconds(5000);
        count_csk++;
        clock_state = !clock_state;
        digitalWrite(RED_LED, clock_state);

      }
      break;

    case 3:
      if (count_frame == NO_FRAMES) {
        count_frame = 0;
        stopTimer();
        state_vlc_csk = 0;
      }
      else state_vlc_csk = 0;
      break;


    default:
      break;

  }



  //Serial.println(count_csk);

}

void vlc_csk_white1000(void)
{

  static uint8_t state_vlc_csk = 0;
  static uint32_t count_csk_white = 0;

  TimerIntClear(TIMERA0_BASE, TimerIntStatus(TIMERA0_BASE, true));

  switch (state_vlc_csk) {

    case 0:
      //sync();
      count_csk_white++;
      break;

    case 1:
      test_csk_white1000();
      count_csk_white++;
      break;

    default:
      break;

  }

  if (count_csk_white == freq_vlc) {
    state_vlc_csk = 1;
    set_OutputDAC(ch_w, 0, 0, 0);
  }

  if (count_csk_white == 10 * freq_vlc) {
    stopTimer();
    count_csk_white = 0;
    state_vlc_csk = 0;
    //Serial.print('F');
  }
}




void sync(void) {

  static boolean state_sync = LOW;

  //digitalWrite(GREEN_LED,state_sync);

  if (state_sync) {
    //rgbw_val[3] = wlevel_ook;
    decimal2hexa(0, 0, 0, 0);
    setAPI_DAC('B');
  }
  else {
    //rgbw_val[3] = 0;
    decimal2hexa(0, 0, 0, wlevel_ook);
    setAPI_DAC('B');
  }

  state_sync = !state_sync;
}

void test_ook(void)
{
  //static boolean data_state = HIGH;

  //Toogle the RED LED in the Lauchpad. This bit control the Synchronization for SER experiments.
  static bool clock_state = LOW;
  digitalWrite(RED_LED, clock_state);

  static boolean data_state = HIGH;
  static uint8_t count_testook = 0;
  //static uint8_t count_ook = 0;
  TimerIntClear(TIMERA0_BASE, TimerIntStatus(TIMERA0_BASE, true));

  clock_state = !clock_state;
  digitalWrite(RED_LED, clock_state);

  if (count_testook == 0) {
    ook_mapping(data_state);
    //data_state = !data_state;
    count_testook++;

    //clock_state = !clock_state;
    //digitalWrite(RED_LED,clock_state);

  }
  else {
    ook_mapping(data_state);
    count_testook = 0;
    data_state = !data_state;

    //clock_state = !clock_state;
    //digitalWrite(RED_LED,clock_state);

  }


}


void test_csk(void)
{
  static uint8_t sym_csk = 0;

  //Toogle the RED LED in the Lauchpad. This bit control the Synchronization for SER experiments.
  static bool clock_state = LOW;

  Serial.print(sym_csk);

  TimerIntClear(TIMERA0_BASE, TimerIntStatus(TIMERA0_BASE, true));

  digitalWrite(RED_LED, clock_state);
  clock_state = !clock_state;

  if (csk_type == '4') {
    xy_16mapping(stack16[sym_csk]);
    if (sym_csk == 15) sym_csk = 0;
    else   sym_csk++;
  }
  if (csk_type == '3') {
    xy_8mapping(stack8[sym_csk]);
    if (sym_csk == 7) sym_csk = 0;
    else   sym_csk++;
  }
  if (csk_type == '2') {
    xy_16mapping(stack4[sym_csk]);
    if (sym_csk == 3) sym_csk = 0;
    else   sym_csk++;
  }



}

void test_csk_white1000(void)
{
  static uint8_t sym_csk = 0;

  TimerIntClear(TIMERA0_BASE, TimerIntStatus(TIMERA0_BASE, true));

  xy_w1000_mapping(sym_csk);
  sym_csk++;
  if (sym_csk == 15) sym_csk = 0;

}


void ook_mapping(boolean ook_sym) {

  if (ook_sym) {
    decimal2hexa(0, 0, 0, wlevel_ook);
    setAPI_DAC('B');
  }

  else {
    decimal2hexa(0, 0, 0, 0);
    setAPI_DAC('B');
  }


}


void xy_8mapping(char buffer_8csk) {
  static uint16_t csk_rgb[3] = {
    0
  };

  switch (buffer_8csk) {
    case '0':
      csk_rgb[0] = Pijk8[1][0];
      csk_rgb[1] = Pijk8[1][1];
      csk_rgb[2] = Pijk8[1][2];
      decimal2hexa(csk_rgb[0], csk_rgb[1], csk_rgb[2], rgbw_val[3]);
      setAPI_DAC('C');
      break;

    case '1':
      csk_rgb[0] = Pijk8[5][0];
      csk_rgb[1] = Pijk8[5][1];
      csk_rgb[2] = Pijk8[5][2];
      decimal2hexa(csk_rgb[0], csk_rgb[1], csk_rgb[2], rgbw_val[3]);
      setAPI_DAC('C');
      break;

    case '2':
      csk_rgb[0] = Pijk8[3][0];
      csk_rgb[1] = Pijk8[3][1];
      csk_rgb[2] = Pijk8[3][2];
      decimal2hexa(csk_rgb[0], csk_rgb[1], csk_rgb[2], rgbw_val[3]);
      setAPI_DAC('C');
      break;

    case '3':
      csk_rgb[0] = Pijk8[6][0];
      csk_rgb[1] = Pijk8[6][1];
      csk_rgb[2] = Pijk8[6][2];
      decimal2hexa(csk_rgb[0], csk_rgb[1], csk_rgb[2], rgbw_val[3]);
      setAPI_DAC('C');
      break;

    case '4':
      csk_rgb[0] = Pijk8[0][0];
      csk_rgb[1] = Pijk8[0][1];
      csk_rgb[2] = Pijk8[0][2];
      decimal2hexa(csk_rgb[0], csk_rgb[1], csk_rgb[2], rgbw_val[3]);
      setAPI_DAC('C');
      break;

    case '5':
      csk_rgb[0] = Pijk8[4][0];
      csk_rgb[1] = Pijk8[4][1];
      csk_rgb[2] = Pijk8[4][2];
      decimal2hexa(csk_rgb[0], csk_rgb[1], csk_rgb[2], rgbw_val[3]);
      setAPI_DAC('C');
      break;

    case '6':
      csk_rgb[0] = Pijk8[2][0];
      csk_rgb[1] = Pijk8[2][1];
      csk_rgb[2] = Pijk8[2][2];
      decimal2hexa(csk_rgb[0], csk_rgb[1], csk_rgb[2], rgbw_val[3]);
      setAPI_DAC('C');
      break;

    case '7':
      csk_rgb[0] = Pijk8[7][0];
      csk_rgb[1] = Pijk8[7][1];
      csk_rgb[2] = Pijk8[7][2];
      decimal2hexa(csk_rgb[0], csk_rgb[1], csk_rgb[2], rgbw_val[3]);
      setAPI_DAC('C');
      break;

    case 'X':
      decimal2hexa(0, 0, 0, 0);
      setAPI_DAC('C');
      break;

    default:
      break;
  }

}




void xy_16mapping(char buffer_16csk) {
  static uint16_t csk_rgb[3] = {
    0
  };

  switch (buffer_16csk) {
    case '0':
      csk_rgb[0] = Pijk[5][0];
      csk_rgb[1] = Pijk[5][1];
      csk_rgb[2] = Pijk[5][2];
      decimal2hexa(csk_rgb[0], csk_rgb[1], csk_rgb[2], rgbw_val[3]);
      setAPI_DAC('C');
      break;

    case '1':
      csk_rgb[0] = Pijk[1][0];
      csk_rgb[1] = Pijk[1][1];
      csk_rgb[2] = Pijk[1][2];
      decimal2hexa(csk_rgb[0], csk_rgb[1], csk_rgb[2], rgbw_val[3]);
      setAPI_DAC('C');
      break;

    case '2':
      csk_rgb[0] = Pijk[4][0];
      csk_rgb[1] = Pijk[4][1];
      csk_rgb[2] = Pijk[4][2];
      decimal2hexa(csk_rgb[0], csk_rgb[1], csk_rgb[2], rgbw_val[3]);
      setAPI_DAC('C');
      break;

    case '3':
      csk_rgb[0] = Pijk[2][0];
      csk_rgb[1] = Pijk[2][1];
      csk_rgb[2] = Pijk[2][2];
      decimal2hexa(csk_rgb[0], csk_rgb[1], csk_rgb[2], rgbw_val[3]);
      setAPI_DAC('C');
      break;

    case '4':
      csk_rgb[0] = Pijk[12][0];
      csk_rgb[1] = Pijk[12][1];
      csk_rgb[2] = Pijk[12][2];
      decimal2hexa(csk_rgb[0], csk_rgb[1], csk_rgb[2], rgbw_val[3]);
      setAPI_DAC('C');
      break;

    case '5':
      csk_rgb[0] = Pijk[3][0];
      csk_rgb[1] = Pijk[3][1];
      csk_rgb[2] = Pijk[3][2];
      decimal2hexa(csk_rgb[0], csk_rgb[1], csk_rgb[2], rgbw_val[3]);
      setAPI_DAC('C');
      break;

    case '6':
      csk_rgb[0] = Pijk[0][0];
      csk_rgb[1] = Pijk[0][1];
      csk_rgb[2] = Pijk[0][2];
      decimal2hexa(csk_rgb[0], csk_rgb[1], csk_rgb[2], rgbw_val[3]);
      setAPI_DAC('C');
      break;

    case '7':
      csk_rgb[0] = Pijk[6][0];
      csk_rgb[1] = Pijk[6][1];
      csk_rgb[2] = Pijk[6][2];
      decimal2hexa(csk_rgb[0], csk_rgb[1], csk_rgb[2], rgbw_val[3]);
      setAPI_DAC('C');
      break;

    case '8':
      csk_rgb[0] = Pijk[15][0];
      csk_rgb[1] = Pijk[15][1];
      csk_rgb[2] = Pijk[15][2];
      decimal2hexa(csk_rgb[0], csk_rgb[1], csk_rgb[2], rgbw_val[3]);
      setAPI_DAC('C');
      break;

    case '9':
      csk_rgb[0] = Pijk[10][0];
      csk_rgb[1] = Pijk[10][1];
      csk_rgb[2] = Pijk[10][2];
      decimal2hexa(csk_rgb[0], csk_rgb[1], csk_rgb[2], rgbw_val[3]);
      setAPI_DAC('C');
      break;

    case 'a':
      csk_rgb[0] = Pijk[8][0];
      csk_rgb[1] = Pijk[8][1];
      csk_rgb[2] = Pijk[8][2];
      decimal2hexa(csk_rgb[0], csk_rgb[1], csk_rgb[2], rgbw_val[3]);
      setAPI_DAC('C');
      break;

    case 'b':
      csk_rgb[0] = Pijk[9][0];
      csk_rgb[1] = Pijk[9][1];
      csk_rgb[2] = Pijk[9][2];
      decimal2hexa(csk_rgb[0], csk_rgb[1], csk_rgb[2], rgbw_val[3]);
      setAPI_DAC('C');
      break;

    case 'c':
      csk_rgb[0] = Pijk[14][0];
      csk_rgb[1] = Pijk[14][1];
      csk_rgb[2] = Pijk[14][2];
      decimal2hexa(csk_rgb[0], csk_rgb[1], csk_rgb[2], rgbw_val[3]);
      setAPI_DAC('C');
      break;

    case 'd':
      csk_rgb[0] = Pijk[13][0];
      csk_rgb[1] = Pijk[13][1];
      csk_rgb[2] = Pijk[13][2];
      decimal2hexa(csk_rgb[0], csk_rgb[1], csk_rgb[2], rgbw_val[3]);
      setAPI_DAC('C');
      break;

    case 'e':
      csk_rgb[0] = Pijk[7][0];
      csk_rgb[1] = Pijk[7][1];
      csk_rgb[2] = Pijk[7][2];
      decimal2hexa(csk_rgb[0], csk_rgb[1], csk_rgb[2], rgbw_val[3]);
      setAPI_DAC('C');
      break;

    case 'f':
      csk_rgb[0] = Pijk[11][0];
      csk_rgb[1] = Pijk[11][1];
      csk_rgb[2] = Pijk[11][2];
      decimal2hexa(csk_rgb[0], csk_rgb[1], csk_rgb[2], rgbw_val[3]);
      setAPI_DAC('C');
      break;

    case 'X':
      decimal2hexa(0, 0, 0, 0);
      setAPI_DAC('C');
      break;

    default:
      break;
  }

}


void xy_w1000_mapping(uint8_t buffer_16csk) {
  static uint16_t csk_rgb[3] = {
    0
  };

  switch (buffer_16csk) {
    case 0:
      csk_rgb[0] = Pijk_w1000[5][0];
      csk_rgb[1] = Pijk_w1000[5][1];
      csk_rgb[2] = Pijk_w1000[5][2];
      decimal2hexa(csk_rgb[0], csk_rgb[1], csk_rgb[2], rgbw_val[3]);
      setAPI_DAC('C');
      break;

    case 1:
      csk_rgb[0] = Pijk_w1000[1][0];
      csk_rgb[1] = Pijk_w1000[1][1];
      csk_rgb[2] = Pijk_w1000[1][2];
      decimal2hexa(csk_rgb[0], csk_rgb[1], csk_rgb[2], rgbw_val[3]);
      setAPI_DAC('C');
      break;

    case 2:
      csk_rgb[0] = Pijk_w1000[4][0];
      csk_rgb[1] = Pijk_w1000[4][1];
      csk_rgb[2] = Pijk_w1000[4][2];
      decimal2hexa(csk_rgb[0], csk_rgb[1], csk_rgb[2], rgbw_val[3]);
      setAPI_DAC('C');
      break;

    case 3:
      csk_rgb[0] = Pijk_w1000[2][0];
      csk_rgb[1] = Pijk_w1000[2][1];
      csk_rgb[2] = Pijk_w1000[2][2];
      decimal2hexa(csk_rgb[0], csk_rgb[1], csk_rgb[2], rgbw_val[3]);
      setAPI_DAC('C');
      break;

    case 4:
      csk_rgb[0] = Pijk_w1000[12][0];
      csk_rgb[1] = Pijk_w1000[12][1];
      csk_rgb[2] = Pijk_w1000[12][2];
      decimal2hexa(csk_rgb[0], csk_rgb[1], csk_rgb[2], rgbw_val[3]);
      setAPI_DAC('C');
      break;

    case 5:
      csk_rgb[0] = Pijk_w1000[3][0];
      csk_rgb[1] = Pijk_w1000[3][1];
      csk_rgb[2] = Pijk_w1000[3][2];
      decimal2hexa(csk_rgb[0], csk_rgb[1], csk_rgb[2], rgbw_val[3]);
      setAPI_DAC('C');
      break;

    case 6:
      csk_rgb[0] = Pijk_w1000[0][0];
      csk_rgb[1] = Pijk_w1000[0][1];
      csk_rgb[2] = Pijk_w1000[0][2];
      decimal2hexa(csk_rgb[0], csk_rgb[1], csk_rgb[2], rgbw_val[3]);
      setAPI_DAC('C');
      break;

    case 7:
      csk_rgb[0] = Pijk_w1000[6][0];
      csk_rgb[1] = Pijk_w1000[6][1];
      csk_rgb[2] = Pijk_w1000[6][2];
      decimal2hexa(csk_rgb[0], csk_rgb[1], csk_rgb[2], rgbw_val[3]);
      setAPI_DAC('C');
      break;

    case 8:
      csk_rgb[0] = Pijk_w1000[15][0];
      csk_rgb[1] = Pijk_w1000[15][1];
      csk_rgb[2] = Pijk_w1000[15][2];
      decimal2hexa(csk_rgb[0], csk_rgb[1], csk_rgb[2], rgbw_val[3]);
      setAPI_DAC('C');
      break;

    case 9:
      csk_rgb[0] = Pijk_w1000[10][0];
      csk_rgb[1] = Pijk_w1000[10][1];
      csk_rgb[2] = Pijk_w1000[10][2];
      decimal2hexa(csk_rgb[0], csk_rgb[1], csk_rgb[2], rgbw_val[3]);
      setAPI_DAC('C');
      break;

    case 10:
      csk_rgb[0] = Pijk_w1000[8][0];
      csk_rgb[1] = Pijk_w1000[8][1];
      csk_rgb[2] = Pijk_w1000[8][2];
      decimal2hexa(csk_rgb[0], csk_rgb[1], csk_rgb[2], rgbw_val[3]);
      setAPI_DAC('C');
      break;

    case 11:
      csk_rgb[0] = Pijk_w1000[9][0];
      csk_rgb[1] = Pijk_w1000[9][1];
      csk_rgb[2] = Pijk_w1000[9][2];
      decimal2hexa(csk_rgb[0], csk_rgb[1], csk_rgb[2], rgbw_val[3]);
      setAPI_DAC('C');
      break;

    case 12:
      csk_rgb[0] = Pijk_w1000[14][0];
      csk_rgb[1] = Pijk_w1000[14][1];
      csk_rgb[2] = Pijk_w1000[14][2];
      decimal2hexa(csk_rgb[0], csk_rgb[1], csk_rgb[2], rgbw_val[3]);
      setAPI_DAC('C');
      break;

    case 13:
      csk_rgb[0] = Pijk_w1000[13][0];
      csk_rgb[1] = Pijk_w1000[13][1];
      csk_rgb[2] = Pijk_w1000[13][2];
      decimal2hexa(csk_rgb[0], csk_rgb[1], csk_rgb[2], rgbw_val[3]);
      setAPI_DAC('C');
      break;

    case 14:
      csk_rgb[0] = Pijk_w1000[7][0];
      csk_rgb[1] = Pijk_w1000[7][1];
      csk_rgb[2] = Pijk_w1000[7][2];
      decimal2hexa(csk_rgb[0], csk_rgb[1], csk_rgb[2], rgbw_val[3]);
      setAPI_DAC('C');
      break;

    case 15:
      csk_rgb[0] = Pijk_w1000[11][0];
      csk_rgb[1] = Pijk_w1000[11][1];
      csk_rgb[2] = Pijk_w1000[11][2];
      decimal2hexa(csk_rgb[0], csk_rgb[1], csk_rgb[2], rgbw_val[3]);
      setAPI_DAC('C');
      break;

    default:
      break;
  }

}

void manchester(void) {

  for (int i = 0; i < L_FRAME / 2; i++) {
    if (buffer_ook[i] == '1') {
      buffer_man[2 * i] = HIGH;
      buffer_man[2 * i + 1] = LOW;
    }

    if (buffer_ook[i] == '0') {
      buffer_man[2 * i] = LOW;
      buffer_man[2 * i + 1] = HIGH;
    }
  }
}
