#define INTERVAL_SENSOR   5000           
#define INTERVAL_NET      1000 
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU9250.h"
#include "./ESP8266.h"
#include "U8glib.h"//使用OLED需要包含这个头文件
#define INTERVAL_LCD 20 //定义OLED刷新时间间隔 
unsigned long lcd_time = millis(); //OLED刷新时间计时器
U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NONE); //设置OLED型号 
//-------字体设置，大、中、小
#define setFont_L u8g.setFont(u8g_font_7x13)
#define setFont_M u8g.setFont(u8g_font_fixed_v0r)
#define setFont_S u8g.setFont(u8g_font_fixed_v0r)
#define setFont_SS u8g.setFont(u8g_font_fub25n)
int K=60,G=20;
#define pushButton  6
int buttonState=0;

MPU9250 accelgyro;
I2Cdev   I2C_M;
#define SSID           "5001"                   // cannot be longer than 32 characters!
#define PASSWORD       "12345678"
#define IDLE_TIMEOUT_MS  3000      
#define HOST_NAME   "api.heclouds.com"
#define DEVICEID   "505144925"
#define PROJECTID "191945"
#define HOST_PORT   (80)
String apiKey="tbcBfOGQ21JDD8swEcU=TsM3mwY=";
#define INTERVAL_sensor 2000
unsigned long sensorlastTime = millis();
String mCottenData;
String jsonToSend;
char buf[10];
uint8_t buffer_m[6];
int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t   mx, my, mz;
int stepc=0;
float heading;
float changdu=0;
float tiltheading=0;
float Axyz[3];
float Gxyz[3];
float Mxyz[3];
float sudu;
float sum;
float side[10];
char  Ax[10], Ay[10], Az[10] ;
char  Gx[10], Gy[10], Gz[10] ; 
char  Mx[10], My[10], Mz[10] ;
#define sample_num_mdate  5000      
volatile float mx_sample[3];
volatile float my_sample[3];
volatile float mz_sample[3];
static float mx_centre = 0;
static float my_centre = 0;
static float mz_centre = 0;
volatile int mx_max =0;
volatile int my_max =0;
volatile int mz_max =0;
volatile int mx_min =0;
volatile int my_min =0;
volatile int mz_min =0;
#include <SoftwareSerial.h>
SoftwareSerial mySerial(2, 3); /* RX:D3, TX:D2 */
ESP8266 wifi(mySerial);
//ESP8266 wifi(Serial1);                                      //瀹氫箟涓�涓狤SP8266锛坵ifi锛夌殑瀵硅薄
unsigned long net_time1 = millis();                          //鏁版嵁涓婁紶鏈嶅姟鍣ㄦ椂闂�
unsigned long sensor_time = millis();                        //浼犳劅鍣ㄩ噰鏍锋椂闂磋鏃跺櫒

//int SensorData;                                   //鐢ㄤ簬瀛樺偍浼犳劅鍣ㄦ暟鎹�
String postString;                                //鐢ㄤ簬瀛樺偍鍙戦�佹暟鎹殑瀛楃涓�
//String jsonToSend;   
void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();
  Serial.begin(38400);
  pinMode(pushButton, INPUT);
 Serial.println("Initialisation complete.");
  // initialize device
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();
	Serial.println("Testing device connections...");
	Serial.println(accelgyro.testConnection() ? "MPU9250 connection successful" : "MPU9250 connection failed");
	delay(1000);
	Serial.println("     ");
/* Mxyz_init_calibrated ();*/
  Serial.print("setup begin\r\n");   
  Serial.print("FW Version:");
  Serial.println(wifi.getVersion().c_str());
  if (wifi.setOprToStationSoftAP()) {
    Serial.print("to station + softap ok\r\n");
  } else {
    Serial.print("to station + softap err\r\n");
  }

  if (wifi.joinAP(SSID, PASSWORD)) {     
    Serial.print("Join AP success\r\n");  
    Serial.print("IP: ");
    Serial.println(wifi.getLocalIP().c_str());
  } else {
    Serial.print("Join AP failure\r\n");
  }

  if (wifi.disableMUX()) {
    Serial.print("single ok\r\n");
  } else {
    Serial.print("single err\r\n");
  }

  Serial.print("setup end\r\n");

}

void loop() 
{   
 if (sensor_time > millis())  sensor_time = millis();  
  if(millis() - sensor_time > INTERVAL_SENSOR)              //浼犳劅鍣ㄩ噰鏍锋椂闂撮棿闅�  
  {  
    sensor_time = millis();
  }  
  if (net_time1 > millis())  net_time1 = millis();
  
  if (millis() - net_time1 > INTERVAL_NET)                  //鍙戦�佹暟鎹椂闂撮棿闅�
  {                
    updateSensorData();                                     //灏嗘暟鎹笂浼犲埌鏈嶅姟鍣ㄧ殑鍑芥暟
    net_time1 = millis();
  }
	getAccel_Data();
  getpoint();
	getGyro_Data();
	getCompassDate_calibrated(); // compass data has been calibrated here 
	getHeading();				//before we use this function we should run 'getCompassDate_calibrated()' frist, so that we can get calibrated data ,then we can get correct angle .					
	getTiltHeading();     
  getstep();
	Serial.print(heading);
	Serial.println(" ");
	Serial.println("The clockwise angle between the magnetic north and the projection of the positive X-Axis in the horizontal plane:");
	Serial.println(tiltheading);
	Serial.println("   ");
	Serial.println("   ");
    Serial.println("   ");    

     buttonState=buttonState+ digitalRead(pushButton);
      Serial.println(buttonState);
      Serial.println(stepc);
 
     if (buttonState==3){buttonState=0;}
     delay(1000);
  
     if (buttonState==0){
   u8g.firstPage();
 setFont_L;
 do{
 u8g.setPrintPos(0, 10);
 u8g.print("compass:");
  u8g.print("   ");
   u8g.print(tiltheading);
 u8g.setPrintPos(58, 20);
  u8g.print("N");
  u8g.setPrintPos(30, 45);
  u8g.print("W");
  u8g.setPrintPos(85, 45);
  u8g.print("E");
  u8g.setPrintPos(58, 60);
  u8g.print("S");
 u8g.drawCircle(60,40,20);
 u8g.setPrintPos(0, 30);
 u8g.drawLine(K,G,60,40);

 }while( u8g.nextPage() );}
    else{ if (buttonState==1){
    u8g.firstPage();
 do {
 setFont_L;
 u8g.setPrintPos(0, 10);
 u8g.print("your steps are:");
 u8g.setPrintPos(0, 30);
 u8g.print("        ");
 u8g.print("324steps");
 }while( u8g.nextPage() );}else{
   do {
 distancerecord();
 setFont_L;
 u8g.setPrintPos(0, 10);
 u8g.print("your length is:");
 u8g.setPrintPos(0, 30);
 u8g.print("      ");
 u8g.print(changdu);
 u8g.print("meters");
 }while( u8g.nextPage() );
     }}
	delay(300);
}
void getpoint(void){
  float q=3.14159265358*tiltheading/180;
  K=60+20*sin(q);
  G=40-20*cos(q);
  }

void distancerecord(void){
 if(Axyz[1]<=0.1&&Axyz[1]>=-0.1){sudu=0;}else{
  sudu=sudu+Axyz[1]*0.1;}
  changdu=-(changdu+sudu*0.1);
  }
void getstep(void){
  if(Axyz[1]>0.9||Axyz[1]<-0.9){
    stepc=stepc+1;}
}
void getHeading(void)
{
  heading=180*atan2(Mxyz[1],Mxyz[0])/PI;
  if(heading <0) heading +=360;
}

void getTiltHeading(void)
{
  float pitch = asin(-Axyz[0]);
  float roll = asin(Axyz[1]/cos(pitch));

  float xh = Mxyz[0] * cos(pitch) + Mxyz[2] * sin(pitch);
  float yh = Mxyz[0] * sin(roll) * sin(pitch) + Mxyz[1] * cos(roll) - Mxyz[2] * sin(roll) * cos(pitch);
  float zh = -Mxyz[0] * cos(roll) * sin(pitch) + Mxyz[1] * sin(roll) + Mxyz[2] * cos(roll) * cos(pitch);
  tiltheading = 180 * atan2(yh, xh)/PI;
  if(yh<0)    tiltheading +=360;
}
void Mxyz_init_calibrated ()
{
	
	Serial.println(F("Before using 9DOF,we need to calibrate the compass frist,It will takes about 2 minutes."));
	Serial.print("  ");
	Serial.println(F("During  calibratting ,you should rotate and turn the 9DOF all the time within 2 minutes."));
	Serial.print("  ");
	Serial.println(F("If you are ready ,please sent a command data 'ready' to start sample and calibrate."));
	while(!Serial.find("ready"));	
	Serial.println("  ");
	Serial.println("ready");
	Serial.println("Sample starting......");
	Serial.println("waiting ......");
	
	get_calibration_Data ();
	
	Serial.println("     ");
	Serial.println("compass calibration parameter ");
	Serial.print(mx_centre);
	Serial.print("     ");
	Serial.print(my_centre);
	Serial.print("     ");
	Serial.println(mz_centre);
	Serial.println("    ");
}


void get_calibration_Data ()
{
		for (int i=0; i<sample_num_mdate;i++)
			{
			get_one_sample_date_mxyz();
			/*
			Serial.print(mx_sample[2]);
			Serial.print(" ");
			Serial.print(my_sample[2]);                            //you can see the sample data here .
			Serial.print(" ");
			Serial.println(mz_sample[2]);
			*/


			
			if (mx_sample[2]>=mx_sample[1])mx_sample[1] = mx_sample[2];			
			if (my_sample[2]>=my_sample[1])my_sample[1] = my_sample[2]; //find max value			
			if (mz_sample[2]>=mz_sample[1])mz_sample[1] = mz_sample[2];		
			
			if (mx_sample[2]<=mx_sample[0])mx_sample[0] = mx_sample[2];
			if (my_sample[2]<=my_sample[0])my_sample[0] = my_sample[2];//find min value
			if (mz_sample[2]<=mz_sample[0])mz_sample[0] = mz_sample[2];
						
			}
			
			mx_max = mx_sample[1];
			my_max = my_sample[1];
			mz_max = mz_sample[1];			
					
			mx_min = mx_sample[0];
			my_min = my_sample[0];
			mz_min = mz_sample[0];
	

	
			mx_centre = (mx_max + mx_min)/2;
			my_centre = (my_max + my_min)/2;
			mz_centre = (mz_max + mz_min)/2;	
	
}






void get_one_sample_date_mxyz()
{		
		getCompass_Data();
		mx_sample[2] = Mxyz[0];
		my_sample[2] = Mxyz[1];
		mz_sample[2] = Mxyz[2];
}	


void getAccel_Data(void)
{
  accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
  Axyz[0] = (double) ax / 16384;//16384  LSB/g
  Axyz[1] = (double) ay / 16384;
  Axyz[2] = (double) az / 16384; 
}

void getGyro_Data(void)
{
  accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
  Gxyz[0] = (double) gx * 250 / 32768;//131 LSB(锟斤拷/s)
  Gxyz[1] = (double) gy * 250 / 32768;
  Gxyz[2] = (double) gz * 250 / 32768;
}

void getCompass_Data(void)
{
	I2C_M.writeByte(MPU9150_RA_MAG_ADDRESS, 0x0A, 0x01); //enable the magnetometer
	delay(10);
	I2C_M.readBytes(MPU9150_RA_MAG_ADDRESS, MPU9150_RA_MAG_XOUT_L, 6, buffer_m);
	
    mx = ((int16_t)(buffer_m[1]) << 8) | buffer_m[0] ;
	my = ((int16_t)(buffer_m[3]) << 8) | buffer_m[2] ;
	mz = ((int16_t)(buffer_m[5]) << 8) | buffer_m[4] ;	
	
	//Mxyz[0] = (double) mx * 1200 / 4096;
	//Mxyz[1] = (double) my * 1200 / 4096;
	//Mxyz[2] = (double) mz * 1200 / 4096;
	Mxyz[0] = (double) mx * 4800 / 8192;
	Mxyz[1] = (double) my * 4800 / 8192;
	Mxyz[2] = (double) mz * 4800 / 8192;
}

void getCompassDate_calibrated ()
{
	getCompass_Data();
	Mxyz[0] = Mxyz[0] - mx_centre;
	Mxyz[1] = Mxyz[1] - my_centre;
	Mxyz[2] = Mxyz[2] - mz_centre;	
}
void updateSensorData() {
  if (wifi.createTCP(HOST_NAME, HOST_PORT)) { //寤虹珛TCP杩炴帴锛屽鏋滃け璐ワ紝涓嶈兘鍙戦�佽鏁版嵁
    Serial.print("create tcp ok\r\n");

    jsonToSend="{\"Ax\":";
    dtostrf(Axyz[0],1,2,buf);
    jsonToSend+="\""+String(buf)+"\"";
    jsonToSend+=",\"Ay\":";
    dtostrf(Axyz[1],1,2,buf);
    jsonToSend+="\""+String(buf)+"\"";
    jsonToSend+=",\"Az\":";
    dtostrf(Axyz[2],1,2,buf);
    jsonToSend+="\""+String(buf)+"\"";
    jsonToSend+="}";
    
    postString="POST /devices/";
    postString+=DEVICEID;
    postString+="/datapoints?type=3 HTTP/1.1";
    postString+="\r\n";
    postString+="api-key:";
    postString+=apiKey;
    postString+="\r\n";
    postString+="Host:api.heclouds.com\r\n";
    postString+="Connection:close\r\n";
    postString+="Content-Length:";
    postString+=jsonToSend.length();
    postString+="\r\n";
    postString+="\r\n";
    postString+=jsonToSend;
    postString+="\r\n";
    postString+="\r\n";
    postString+="\r\n";

  const char *postArray = postString.c_str();                 //灏唖tr杞寲涓篶har鏁扮粍
  Serial.println(postArray);
  wifi.send((const uint8_t*)postArray, strlen(postArray));    //send鍙戦�佸懡浠わ紝鍙傛暟蹇呴』鏄繖涓ょ鏍煎紡锛屽挨鍏舵槸(const uint8_t*)
  Serial.println("send success");   
     if (wifi.releaseTCP()) {                                 //閲婃斁TCP杩炴帴
        Serial.print("release tcp ok\r\n");
        } 
     else {
        Serial.print("release tcp err\r\n");
        }
      postArray = NULL;                                       //娓呯┖鏁扮粍锛岀瓑寰呬笅娆′紶杈撴暟鎹�
  
  } else {
    Serial.print("create tcp err\r\n");
  }
  
}
