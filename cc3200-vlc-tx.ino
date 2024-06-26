  

/*
    Webserver and Wifi network application for Visible Light Communicaiton
  
  Designed by Juan Felipe Gutiérrez Gómez
  email: jufgutierrezgo@gmail.com
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

#define NO_FRAMES 250

#define L_FRAME 240

#define L_MESSAGE_CSK 25

#define stack16 "0123456789ABCDEF"
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
char message_CSK[] = "CSK message: Hello World!";
//const String VLC_Message_CSK = String("*809809fedcba9876543210fedcba9876543210fedcba9876543210fedcba9876543210@@@@");
//const  String VLC_Message_OOK = String("+++VLC****@jufgutierrezgo*****optical-link-on-off-keying++++");
//const  String VLC_Message_OOK = String("VLC@jufgutierrezgo**mod-ook***");
const  String VLC_Message_OOK = String("This-is-a-new-message-jfg2****");
//char buffer_csk[] = {"2a2a2a2a2a2a2a564c43406a756667757469657272657a676f2a2a2a2a2a2a6d61737465722d696e2d736369656e63652d6f662d74656c65636f6d756e69636174696f6e2a2a2a2a2a2a2a"};
//char buffer_16csk[] = {"XX8090123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef0123456789a"};
//This array is used when the receiver only read 80 symbols (240 adc-data values)
//char buffer_16csk[] = {"XX8090123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef0123456789a"};
char buffer_16csk[2*L_MESSAGE_CSK+2];
char buffer_8csk[] =  {"XX745012345670123456701234567012345670123456701234567012345670123456701234567012"};
char buffer_4csk[] =  {"XX809680968096809680968096809680968096809680968096809680968096809680968096809680"};

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
boolean flag_msg = LOW;

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
  /*
  //Using currently linear regression WHITE 0 L=5cd/m2
  //Computed from power matrix for 16-CSK Constellations
  // Calibration LABE 10-Feb-2023
  const uint16_t Pijk[16][3] = {
  355, 103, 490,
  118, 240, 163,
  0, 206, 490,
  355, 206,   0,
  118, 137, 654,
  0, 309,   0,
  473, 137, 163,
  473,  34, 654,
  0, 103, 980,
  118,  34, 1144,
  0,   0, 1471,
  355,   0, 980,
  709, 103,   0,
  827,  34, 163,
  709,   0, 490,
  1064,   0,   0
  };
*/
// Calibration LABE 31-Mar-2023
const uint16_t Pijk[16][3] = {
  209,  67, 305,
  70, 155, 102,
  0, 133, 305,
  209, 133,   0,
  70,  89, 407,
  0, 200,   0,
  279,  89, 102,
  279,  22, 407,
  0,  67, 611,
  70,  22, 712,
  0,   0, 916,
  209,   0, 611,
  419,  67,   0,
  489,  22, 102,
  419,   0, 305,
  628,   0,   0
};
/*
  //Using currently linear regression WHITE 0 L=6cd/m2
  //Computed from power matrix for 16-CSK Constellations
  const uint16_t Pijk8[8][3] = {
  0, 711,   0,
  0, 474, 610,
  847, 474,   0,
  1553, 197, 203,
  0,   0, 1829,
  282, 197, 1118,
  1271,   0, 915,
  2541,   0,   0
  };
  /*

  //Using currently linear regression WHITE 0 L=5cd/m2
  //Computed from power matrix for 16-CSK Constellations
  // Calibration LABE 10-Feb-2023
  const uint16_t Pijk8[8][3] = {
  0, 309,   0,
  0, 206, 490,
  355, 206,   0,
  650,  86, 163,
  0,   0, 1471,
  118,  86, 899,
  532,   0, 735,
  1064,   0,   0
  };
*/
// Calibration LABE 31-Mar-2023
const uint16_t Pijk8[8][3] = {
  0, 200,   0,
  0, 133, 305,
  209, 133,   0,
  384,  55, 102,
  0,   0, 916,
  70,  55, 560,
  314,   0, 458,
  628,   0,   0
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
  setDAC();



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
  // Convert Message[] to HEX-frame for 16CSK
  // Check the default Message[] 
  
  create_HEX_frame();
    
  //*****************************************************/
  // Convert the character data to bin
  //
  /*
    for (int i = 0; i < sizeof(buffer_ook); i++) buffer_ook[i] = '0';

    for (int i = 0; i < VLC_Message_OOK.length(); i++) {


    String aux2 = String(VLC_Message_OOK[i], BIN);

    //Serial.println(aux2);

    for (int j = aux2.length(); j > 0; j--) buffer_ook[8 * i + (8 - j)] = aux2[aux2.length() - j];

    //Serial.println(VLC_Message_CSK.length());
    }

  */

  /*******************************************************/

  manchester();


  // Print the buffers of the modulations
  /*
    Serial.println("Print buffers:");
    Serial.println(buffer_4csk);
    Serial.println(buffer_ook);
    Serial.println(buffer_man);
  */
}

unsigned int num_clients = 2;

void loop()
{



  unsigned int a, i;

  // ********************* WEB PAGE FUNCTIONS *********************

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
            client.println("");

            // the content of the HTTP response follows the header:
            client.println("<html><head><title>LED-Fixture CC3200</title>");
            client.println("<style type=\"text/css\">");
            client.println("body{margin:0;padding:0;font-family: Sans-Serif;line-height: 1.5em;}");
            client.println(".button{display: inline-block; width: 50%; padding: 5px 25px; font-size: 15px; cursor: pointer; text-align: center; text-decoration: none; outline: none; color: #000; background-color: #ccc; border: none; border-radius: 15px; box-shadow: 0 9px #999;}");
            client.println(".button:hover{background-color: #3e8e41}");
            client.println(".button:active{background-color: #3e8e41; box-shadow: 0 5px #666; transform: translateY(4px);}");
            client.println(".input {width: 50%;}");
            client.println(".selector {width: 50%;}");
            client.println("#chart svg{height: 400px;}");
            client.println("#header{background: #ccc;height: 150px;}");
            client.println("#header h1{margin: 0;padding-top: 15px;}");
            client.println("main{padding-bottom: 10010px;margin-bottom: -10000px;float: left;width: 100%;}");
            client.println("#nav{padding-bottom: 10010px;margin-bottom: -10000px;float: left;width: 200px;margin-left: -100%;background: #eee;}");
            client.println("#footer{clear: left;width: 100%;background: #ccc;text-align: center;padding: 4px 0;}");
            client.println("#wrapper{overflow: hidden;}#content{margin-left: 230px; /* Same as 'nav' width */}.innertube{margin: 25px; /* Padding for content */margin-top: 0;}p{color: #555;}");
            client.println("nav ul{list-style-type: none;margin: 0;padding: 0;}");
            client.println("nav ul a{color: darkgreen;text-decoration: none;}");
            client.println("</style>");

            //Header of the page
            client.println(" <body><header id=\"header\"><div class=\"innertube\">");
            client.println("<h1>WebServer for CC3200 LED-Fixture</h1>");
            //client.println("<h2>Zybyn Technology</h2>");
            client.println("<h3>Designed by Juan F. Gutierrez</h3>");
            client.println("</div></header>");

            //Description content
            client.println("<div id=\"wrapper\">");
            client.println("<main>");
            client.println("<div id=\"content\">");
            client.println("<div class=\"innertube\">");
            client.println("<h1>Description of LED-Fixture</h1>");
            client.println("<p>The webpage of CC3200 LED-Fixture allows to the users control different features for Visible Light Communications. </p>");

            //Channel setter content
            client.println("<h2>Channel Setter</h1>");
            client.println("<iframe name='votar' style='display:none;'></iframe>");
            client.println("<form action='/set_rgbw_page.php' method='HEAD' target='votar'>");
            client.println("RED CHANNEL  <br>");
            client.println("<input class=\"input \" type='number' name='rval' min='0' max='3000' value='0'> <br>");
            client.println("GREEN CHANNEL  <br>");
            client.println("<input class=\"input \" type='number' name='gval' min='0' max='3000' value='0'> <br>");
            client.println("BLUE CHANNEL  <br>");
            client.println("<input class=\"input \" type='number' name='bval' min='0' max='3000' value='0'> <br>");
            client.println("WHITE CHANNEL  <br>");
            client.println("<input class=\"input \" type='number' name='wval' min='0' max='3000' value='0'> <br><br>");
            client.println("<button class=\"button \" onclick='myFunction()' >SET</button><br><br>");
            client.println("</form>");

            //16-CSK Symbol content
            client.println("<h2>16-CSK Symbol Setter</h1>");
            client.println("<iframe name='csk-symbol-16' style='display:none;'></iframe>");
            client.println("<form action='/csk_symbol_16.php' method='HEAD' target='csk-symbol-8'>");
            client.print(" <select class=\"selector \" name='symbol'>");
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
            client.print("<button class=\"button \" onclick=''>SET</button><br>");
            client.println("</form>");

            //8-CSK Symbol content
            client.println("<h2>8-CSK Symbol Setter</h1>");
            client.println("<iframe name='csk-symbol-8' style='display:none;'></iframe>");
            client.println("<form action='/csk_symbol_8.php' method='HEAD' target='csk-symbol-8'>");
            client.print(" <select class=\"selector \" name='symbol'>");
            client.print("<option value='04'>S0-100</option>");
            client.print("<option value='00'>S1-000</option>");
            client.print("<option value='06'>S2-110</option>");
            client.print("<option value='02'>S3-010</option>");
            client.print("<option value='05'>S4-101</option>");
            client.print("<option value='01'>S5-001</option>");
            client.print("<option value='03'>S6-011</option>");
            client.print("<option value='07'>S7-111</option>");
            client.print("</select><br><br>");
            client.print("<button class=\"button \" onclick=''>SET</button><br>");
            client.println("</form>");

            //Frequency setter content
            client.println("<h2>Frequency Setter</h1>");
            client.println("<iframe name='freq' style='display:none;'></iframe>");
            client.println("<form action='/set_freq.php' method='HEAD' target='freq'>");
            client.print("<input class=\"input \" class=\"input \" type='number' name='fval' min='1' max='1000000' value='1000'> <br><br>");
            client.print("<button class=\"button \" onclick='' >Set Freq</button><br>");
            client.println("</form>");

            //Modulation setter content
            client.println("<h2>Modulation Setter</h1>");
            client.print("Choose the modulation <br>");
            client.println("<iframe name='modulation' style='display:none;'></iframe>");
            client.println("<form action='/set_modulation.php' method='HEAD' target='modulation'>");
            client.print(" <select class=\"selector \" name='mod'>");
            client.print("<option value='OOK'>OOK</option>");
            client.print("<option value='4CSK'>4CSK</option>");
            client.print("<option value='8CSK'>8CSK</option>");
            client.print("<option value='16CSK'>16CSK</option>");
            client.print("<option value='testOOK'>testOOK</option>");
            client.print("<option value='test16CSK'>test16CSK</option>");
            client.print("<option value='test8CSK'>test8CSK</option>");
            client.print("<option value='test4CSK'>test4CSK</option>");
            client.print("</select><br><br>");
            client.print("<button class=\"button \" onclick=''>DOWNLOAD</button><br>");
            client.println("</form>");

            //Button setter modulation
            client.println("<iframe name='stop_mod' style='display:none;'></iframe>");
            client.println("<form action='/stop_mod.php' method='HEAD' target='stop_mod'>");
            client.print("<button class=\"button \" onclick=''>STOP</button><br>");
            client.println("</form>");

            //Message setter content
            client.println("<h2>Message Setter</h1>");
            client.println("<iframe name='msg' style='display:none;'></iframe>");
            client.println("<form action='/set_message.php' method='HEAD' target='msg'>");
            client.println("Message to transmit <br>");
            client.println("<input class=\"input \" type='text' name='msg' value='Type your message' maxlength='25'> <br>");            
            client.println("<button class=\"button \" onclick='' >SET</button><br><br>");
            client.println("</form>");

            client.println("<p id='demo'></p>");
            client.println("<script>");
            client.println("function myFunction() {");
            //client.println("document.getElementById('demo').innerHTML = 43;");
            client.println("}");
            client.println("</script>");

            client.println("</div>");
            client.println("</div>");
            client.println("</main>");

            //Navigation
            client.println("<nav id=\"nav\"><div class=\"innertube\"><img src=''/><h3>Sections</h3>");
            client.println("<ul>");
            client.println("<li><a href=\"#\">Channel Setter</a></li>");
            client.println("<li><a href=\"#\">CSK Symbol Setter</a></li>");
            client.println("<li><a href=\"#\">Frequency Setter</a></li>");
            client.println("<li><a href=\"#\">Modulation Setter</a></li> ");
            client.println("</ul></div></nav>");
            client.println("</div>");

            client.println("<footer id=\"footer\">");
            client.println("<div class=\"innertube\">");
            client.println("<p>Zybyn Technology</p>");
            client.println("</div></footer>");

            client.println("</body></html>");

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
        if (endsWith(buffer, "GET /set_message.php")) { // GET /set_rgbw_page.php for set the RGBW channels inn the luminaire
          flag_msg = HIGH;
        }

      }
    }

    //Serial.println(buffer_temp);
    //Serial.println("XYZ");

    if (flag_luminaire) {
      get_valRGBW(buffer_temp);
      decimal2hexa(rgbw_val[0], rgbw_val[1], rgbw_val[2], rgbw_val[3]);
      setOUT_DAC('A');
      flag_luminaire = LOW;
    }

    if (flag_frequency) {
      get_freq(buffer_temp);
      flag_frequency = LOW;
    }

    if (flag_start_vlc) {
      flag_start_vlc = LOW;
      startTimer();
    }

    if (flag_16csk) {
      get_sym16csk(buffer_temp);
      xy_16mapping(stack16[sym]);
      flag_16csk = LOW;

    }
    if (flag_8csk) {
      get_sym16csk(buffer_temp);
      xy_8mapping(stack8[sym]);
      flag_8csk = LOW;

    }
    if (flag_msg) {
      get_msg(buffer_temp);      
      create_HEX_frame();
      flag_msg=LOW;

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


// Convert the Message to HEX frame
void create_HEX_frame(void){    
  
  //converting ascii string to hex string
  char buffer_temp[2*L_MESSAGE_CSK];
  string2hexString(message_CSK, buffer_temp);

  Serial.println("16CSK buffer_TEMP in HEX:");
  Serial.println(buffer_temp);
  
  for(int i=0; i<=sizeof(buffer_16csk); i++){

    if(i<2)
      buffer_16csk[i] = 'X';        
    else 
      buffer_16csk[i] = buffer_temp[i-2];       
      
    }

   
  
  Serial.println("16CSK buffer in HEX:");
  Serial.println(buffer_16csk);

}

//function to convert ascii char[] to hex-string (char[])
void string2hexString(char* input, char* output)
{
    int loop;
    int i;

    i = 0;
    loop = 0;

    while (input[loop] != '\0') {
        sprintf((char*)(output + i), "%02X", input[loop]);
        loop += 1;
        i += 2;
    }
    //insert NULL at the end of the output string
    output[i++] = '\0';
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

// Function to get the message from http 
void get_msg(char* inString) {

   
  //Serial.println(inString);

  int strLength = strlen(inString);
  int h = 0;
  int j = 0;
  int k = 0;
  int count_read = 0;
  boolean copy_char = LOW;
  memset(message_CSK, 0, L_MESSAGE_CSK);

  
  while (h < strLength) {

    if (copy_char) {
      message_CSK[j] =  inString[h];
      j++;
      }
      
    if (inString[h] == '=') copy_char = HIGH;

    if (inString[h+2] == 'H') {
      copy_char = LOW;
      h = strLength;    
    }
    
    h++;
    
  }

  //Serial.print("NANIA");

  //Serial.println(temp_val);
  Serial.println("Message setter:");
  Serial.println(message_CSK);

  // Convert Message to HEX-frame for 16CSK  
  
  memset(buffer_temp, 0, 150);
  //return rgbw_val;
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

  //Serial.print("NANIA");

  //Serial.println(temp_val);
  Serial.println("RGBW values setter:");
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

  Serial.println("Frequency setter:");
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

  Serial.println("16-CSK Symbol setter:");
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

void setDAC() {

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

void setOUT_DAC(char sel) {
  // sel is the variable to choose the configuration of the DAC
  // sel=A-> Control the luminous flux RGBW
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
      state_vlc_csk = 2;

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

      if (count_csk == sizeof(buffer_16csk)) {
        count_frame++;
        count_csk = 0;
        state_vlc_csk = 3;
      }
      else {
        //xy_16mapping is used for 4mapping.
        if (csk_type == '2') xy_16mapping(buffer_4csk[count_csk]);
        if (csk_type == '3') xy_8mapping(buffer_8csk[count_csk]);
        if (csk_type == '4') xy_16mapping(buffer_16csk[count_csk]);
        // delayMicroseconds(5000);
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



void sync(void) {

  static boolean state_sync = LOW;

  //digitalWrite(GREEN_LED,state_sync);

  if (state_sync) {
    //rgbw_val[3] = wlevel_ook;
    decimal2hexa(0, 0, 0, 0);
    setOUT_DAC('B');
  }
  else {
    //rgbw_val[3] = 0;
    decimal2hexa(0, 0, 0, wlevel_ook);
    setOUT_DAC('B');
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
  static int8_t sym_csk = -1;

  //Toogle the RED LED in the Lauchpad. This bit control the Synchronization for SER experiments.
  static bool clock_state = LOW;

  //Serial.print("Symbols transmitted for test:");
  //Serial.print(sym_csk);

  TimerIntClear(TIMERA0_BASE, TimerIntStatus(TIMERA0_BASE, true));

  digitalWrite(RED_LED, clock_state);
  clock_state = !clock_state;

  if (csk_type == '4') {

    if (sym_csk == -1) xy_16mapping('X');
    else xy_16mapping(stack16[sym_csk]);

    if (sym_csk == 15) sym_csk = -1;
    else   sym_csk++;

  }

  if (csk_type == '3') {
    if (sym_csk == -1) xy_8mapping('X');
    else xy_8mapping(stack8[sym_csk]);

    if (sym_csk == 7) sym_csk = -1;
    else   sym_csk++;
  }

  if (csk_type == '2') {
    if (sym_csk == -1) xy_16mapping('X');
    else xy_16mapping(stack4[sym_csk]);

    if (sym_csk == 3) sym_csk = -1;
    else   sym_csk++;
  }

}



void ook_mapping(boolean ook_sym) {

  if (ook_sym) {
    decimal2hexa(0, 0, 0, wlevel_ook);
    setOUT_DAC('B');
  }

  else {
    decimal2hexa(0, 0, 0, 0);
    setOUT_DAC('B');
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
      setOUT_DAC('C');
      break;

    case '1':
      csk_rgb[0] = Pijk8[5][0];
      csk_rgb[1] = Pijk8[5][1];
      csk_rgb[2] = Pijk8[5][2];
      decimal2hexa(csk_rgb[0], csk_rgb[1], csk_rgb[2], rgbw_val[3]);
      setOUT_DAC('C');
      break;

    case '2':
      csk_rgb[0] = Pijk8[3][0];
      csk_rgb[1] = Pijk8[3][1];
      csk_rgb[2] = Pijk8[3][2];
      decimal2hexa(csk_rgb[0], csk_rgb[1], csk_rgb[2], rgbw_val[3]);
      setOUT_DAC('C');
      break;

    case '3':
      csk_rgb[0] = Pijk8[6][0];
      csk_rgb[1] = Pijk8[6][1];
      csk_rgb[2] = Pijk8[6][2];
      decimal2hexa(csk_rgb[0], csk_rgb[1], csk_rgb[2], rgbw_val[3]);
      setOUT_DAC('C');
      break;

    case '4':
      csk_rgb[0] = Pijk8[0][0];
      csk_rgb[1] = Pijk8[0][1];
      csk_rgb[2] = Pijk8[0][2];
      decimal2hexa(csk_rgb[0], csk_rgb[1], csk_rgb[2], rgbw_val[3]);
      setOUT_DAC('C');
      break;

    case '5':
      csk_rgb[0] = Pijk8[4][0];
      csk_rgb[1] = Pijk8[4][1];
      csk_rgb[2] = Pijk8[4][2];
      decimal2hexa(csk_rgb[0], csk_rgb[1], csk_rgb[2], rgbw_val[3]);
      setOUT_DAC('C');
      break;

    case '6':
      csk_rgb[0] = Pijk8[2][0];
      csk_rgb[1] = Pijk8[2][1];
      csk_rgb[2] = Pijk8[2][2];
      decimal2hexa(csk_rgb[0], csk_rgb[1], csk_rgb[2], rgbw_val[3]);
      setOUT_DAC('C');
      break;

    case '7':
      csk_rgb[0] = Pijk8[7][0];
      csk_rgb[1] = Pijk8[7][1];
      csk_rgb[2] = Pijk8[7][2];
      decimal2hexa(csk_rgb[0], csk_rgb[1], csk_rgb[2], rgbw_val[3]);
      setOUT_DAC('C');
      break;

    case 'X':
      decimal2hexa(0, 0, 0, 0);
      setOUT_DAC('C');
      break;

    default:
      break;
  }

}




void xy_16mapping(char symbol_16csk) {
  static uint16_t csk_rgb[3] = {
    0
  };

  switch (symbol_16csk) {
    case '0':
      csk_rgb[0] = Pijk[5][0];
      csk_rgb[1] = Pijk[5][1];
      csk_rgb[2] = Pijk[5][2];
      decimal2hexa(csk_rgb[0], csk_rgb[1], csk_rgb[2], rgbw_val[3]);
      setOUT_DAC('C');
      break;

    case '1':
      csk_rgb[0] = Pijk[1][0];
      csk_rgb[1] = Pijk[1][1];
      csk_rgb[2] = Pijk[1][2];
      decimal2hexa(csk_rgb[0], csk_rgb[1], csk_rgb[2], rgbw_val[3]);
      setOUT_DAC('C');
      break;

    case '2':
      csk_rgb[0] = Pijk[4][0];
      csk_rgb[1] = Pijk[4][1];
      csk_rgb[2] = Pijk[4][2];
      decimal2hexa(csk_rgb[0], csk_rgb[1], csk_rgb[2], rgbw_val[3]);
      setOUT_DAC('C');
      break;

    case '3':
      csk_rgb[0] = Pijk[2][0];
      csk_rgb[1] = Pijk[2][1];
      csk_rgb[2] = Pijk[2][2];
      decimal2hexa(csk_rgb[0], csk_rgb[1], csk_rgb[2], rgbw_val[3]);
      setOUT_DAC('C');
      break;

    case '4':
      csk_rgb[0] = Pijk[12][0];
      csk_rgb[1] = Pijk[12][1];
      csk_rgb[2] = Pijk[12][2];
      decimal2hexa(csk_rgb[0], csk_rgb[1], csk_rgb[2], rgbw_val[3]);
      setOUT_DAC('C');
      break;

    case '5':
      csk_rgb[0] = Pijk[3][0];
      csk_rgb[1] = Pijk[3][1];
      csk_rgb[2] = Pijk[3][2];
      decimal2hexa(csk_rgb[0], csk_rgb[1], csk_rgb[2], rgbw_val[3]);
      setOUT_DAC('C');
      break;

    case '6':
      csk_rgb[0] = Pijk[0][0];
      csk_rgb[1] = Pijk[0][1];
      csk_rgb[2] = Pijk[0][2];
      decimal2hexa(csk_rgb[0], csk_rgb[1], csk_rgb[2], rgbw_val[3]);
      setOUT_DAC('C');
      break;

    case '7':
      csk_rgb[0] = Pijk[6][0];
      csk_rgb[1] = Pijk[6][1];
      csk_rgb[2] = Pijk[6][2];
      decimal2hexa(csk_rgb[0], csk_rgb[1], csk_rgb[2], rgbw_val[3]);
      setOUT_DAC('C');
      break;

    case '8':
      csk_rgb[0] = Pijk[15][0];
      csk_rgb[1] = Pijk[15][1];
      csk_rgb[2] = Pijk[15][2];
      decimal2hexa(csk_rgb[0], csk_rgb[1], csk_rgb[2], rgbw_val[3]);
      setOUT_DAC('C');
      break;

    case '9':
      csk_rgb[0] = Pijk[10][0];
      csk_rgb[1] = Pijk[10][1];
      csk_rgb[2] = Pijk[10][2];
      decimal2hexa(csk_rgb[0], csk_rgb[1], csk_rgb[2], rgbw_val[3]);
      setOUT_DAC('C');
      break;

    case 'A':
      csk_rgb[0] = Pijk[8][0];
      csk_rgb[1] = Pijk[8][1];
      csk_rgb[2] = Pijk[8][2];
      decimal2hexa(csk_rgb[0], csk_rgb[1], csk_rgb[2], rgbw_val[3]);
      setOUT_DAC('C');
      break;

    case 'B':
      csk_rgb[0] = Pijk[9][0];
      csk_rgb[1] = Pijk[9][1];
      csk_rgb[2] = Pijk[9][2];
      decimal2hexa(csk_rgb[0], csk_rgb[1], csk_rgb[2], rgbw_val[3]);
      setOUT_DAC('C');
      break;

    case 'C':
      csk_rgb[0] = Pijk[14][0];
      csk_rgb[1] = Pijk[14][1];
      csk_rgb[2] = Pijk[14][2];
      decimal2hexa(csk_rgb[0], csk_rgb[1], csk_rgb[2], rgbw_val[3]);
      setOUT_DAC('C');
      break;

    case 'D':
      csk_rgb[0] = Pijk[13][0];
      csk_rgb[1] = Pijk[13][1];
      csk_rgb[2] = Pijk[13][2];
      decimal2hexa(csk_rgb[0], csk_rgb[1], csk_rgb[2], rgbw_val[3]);
      setOUT_DAC('C');
      break;

    case 'E':
      csk_rgb[0] = Pijk[7][0];
      csk_rgb[1] = Pijk[7][1];
      csk_rgb[2] = Pijk[7][2];
      decimal2hexa(csk_rgb[0], csk_rgb[1], csk_rgb[2], rgbw_val[3]);
      setOUT_DAC('C');
      break;

    case 'F':
      csk_rgb[0] = Pijk[11][0];
      csk_rgb[1] = Pijk[11][1];
      csk_rgb[2] = Pijk[11][2];
      decimal2hexa(csk_rgb[0], csk_rgb[1], csk_rgb[2], rgbw_val[3]);
      setOUT_DAC('C');
      break;

    case 'X':
      decimal2hexa(0, 0, 0, 0);
      setOUT_DAC('C');
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
