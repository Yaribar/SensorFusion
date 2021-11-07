#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <PubSubClient.h>
#include "ADC_Read.h"
#include <Kalman_Filter.h>
#include <BasicLinearAlgebra.h>
#include "MPU6050_tockn.h"
#include <HCSR04.h>

//******************************************************************************************************************
//*********** MQTT CONFIG ******************************************************************************************
//******************************************************************************************************************
const char *mqtt_server = "";
const int mqtt_port = 1883;
const char *mqtt_user = "";
const char *mqtt_pass = "";
const char *root_topic_subscribe = "";
const char *root_topic_publish = "";

//**************************************
//*********** VAR SENSOR ***************
//**************************************
ADC Voltage, Current;
const byte VOLTAGE_PIN = 35;
const byte CURRENT_PIN = 34;
uint16_t reading1;
uint16_t reading2;

//**************************************
//*********** WIFICONFIG ***************
//**************************************

//const char *ssid = "TELECABLE-7C10";
//const char *password = "GPON00257C10";

const char *ssid = "";
const char *password = "";

//**************************************
//*********** GLOBALES   ***************
//**************************************

WiFiClient espClient;
PubSubClient client(espClient);
char msg[25];
long count = 0;

//************************
//** F U N C I O N E S ***
//************************
void callback(char *topic, byte *payload, unsigned int length);
void reconnect();
void setup_wifi();

//******************************************************************************************************************
//*********** END MQTT CONFIG **************************************************************************************
//******************************************************************************************************************

//******************************************************************************************************************
//*********** KALMAN ***********************************************************************************************
//******************************************************************************************************************
using namespace BLA;
Kalman kalmanazo;

float dt=0.2;
float position=0;
float position_prev=0;
float velocity=0;
float total_distance=0;
float total_distance2=0;

 
uint16_t sample_period_imu = 1; //in milliseconds
uint16_t sample_period_hc = 25; //in milliseconds
uint16_t sample_period = 2000;
uint16_t sample_period_lap= 45000;
bool got_hc, got_imu;

ulong start_time_imu;
ulong start_time_hc;
ulong start_time;
ulong start_time_lap;

float lap=0.0;

MPU6050 mpu6050(Wire);
float accX, accY, accZ;

double* distances;

//******************************************************************************************************************
//*********** HCSR04 ***********************************************************************************************
//******************************************************************************************************************

#define TRIG_PIN 13
#define ECHO_PIN 12

//******************************************************************************************************************
//*********** I2C ***********************************************************************************************
//******************************************************************************************************************

#define I2C_SDA 33
#define I2C_SCL 32

void setup()
{
	
		Serial.begin(115200);
	setup_wifi();
	client.setServer(mqtt_server, mqtt_port);
	client.setCallback(callback);

	HCSR04.begin(TRIG_PIN,ECHO_PIN);

	//******************************************************************************************************************
	//*********** SETUP DE KALMANAZO **************************************************************************
	//******************************************************************************************************************
	BLA::Matrix<2,2> F;
	F << 1, dt,
			0, 1;
	
	BLA::Matrix<2,1> G;
	G << 0,
		dt;

	BLA::Matrix<1,1> u;
	u << 0;

	BLA::Matrix<2,2> Pk_1;
	Pk_1 << 0.5, 0,
			0, 0.5;
	
	BLA::Matrix<2,1> Xk_1;
	Xk_1 << 0,
			0;

	BLA::Matrix<2,2> Q;
	Q << 0.1, 0,
			0, 0.1;


	Voltage.begin(VOLTAGE_PIN, 12, 3.3F);
	Current.begin(CURRENT_PIN, 12, 3.3F);
	got_hc = false;
	got_imu = false;
	start_time_imu = millis();
	start_time_hc = millis();

	Wire.begin(I2C_SDA,I2C_SCL);
	mpu6050.begin();
	mpu6050.calcGyroOffsets(true);

	distances = HCSR04.measureDistanceCm();

	position_prev=distances[0]-1.0;
}

//******************************************************************************************************************
//*********** MAIN *************************************************************************************************
//******************************************************************************************************************

void loop()
{

	if (!client.connected())
	{
		reconnect();
	}

	if (client.connected())
	{
		/*
	String str = "La cuenta es -> " + String(count);
    str.toCharArray(msg,25);
    client.publish(root_topic_publish,msg);
    count++;
	*/
		distances = HCSR04.measureDistanceCm();

		if (millis() - start_time_imu > sample_period_imu)
		{
			mpu6050.update();
			accX = mpu6050.getAccX();
			accY = mpu6050.getAccY();
			accZ = mpu6050.getAccZ();
			//leer imu
			start_time_imu = millis();
			got_imu = true;
		}

		if (millis() - start_time_hc > sample_period_hc)
		{
			//leer ultrasonico
			

			start_time_hc = millis();
			got_hc = true;
		}

		if (got_hc && got_imu)
		{
			//perform sensor fusión
		}

		else if (got_hc)
		{
			//asignar posición mediante hc
		}

		else if (got_imu)
		{
			//asignar posición por imu
		}

		if (millis() - start_time > sample_period)
		{
			Serial.print("1: ");
  			Serial.print(distances[0]-1.0);
  			Serial.println(" cm");
			Serial.print("2: ");
			Serial.print(-1*accY*100.0);
			Serial.println(" cm/s^2");

			position=position_prev-distances[0];
			position_prev=distances[0];
			velocity=position/dt;
			//x+dt
			BLA::Matrix<2,2> F;
			F << 1, dt,
					0, 1;
			
			//dt for acceleration
			BLA::Matrix<2,1> G;
			G << 0,
				dt;

			//Acceleration
			BLA::Matrix<1,1> u;
			u << -1*accY*100.0;

			//Error
			BLA::Matrix<2,2> Pk_1;
			Pk_1 << 0.5, 0,
					0, 4.0;
			
			//position and velocity
			BLA::Matrix<2,1> Xk_1;
			Xk_1 << 0,
					velocity;

			//uncertainty from enviroment
			BLA::Matrix<2,2> Q;
			Q << 0.1, 0,
					0, 0.1;

			BLA::Matrix<2,1> xk_pred = kalmanazo.xk_pred(F, G, Xk_1, u);
			BLA::Matrix<2,2> P = kalmanazo.Pk_pred(F, Pk_1, Q);

			Serial << "Xk: " << xk_pred << '\n';
  			Serial << "Pk_predict" << P << '\n';
			total_distance=total_distance+xk_pred(0);
			Serial.println(total_distance);
			//Serial.print("voltage: ");
			//Serial.print((Voltage.readRaw()*5.0)/4095.0);
			//Serial.print(" Current: ");
			//Serial.println((Current.readRaw()*5.0)/(4095.0));

			BLA::Matrix<1,2> H;
			H << 1.0, 0;

			BLA::Matrix<1,1> y;
			y << position;

			BLA::Matrix<1,1> R;
			R << 0.5;
			
			BLA::Matrix<2,1> k = kalmanazo.K(P,H,R);
			BLA::Matrix<2,1> x_correc = kalmanazo.xk_correc(xk_pred,k,y,H);

			Serial << "k_correc: " << k << '\n';
  			Serial << "x_correc" << x_correc << '\n';
						
			total_distance2=total_distance2+x_correc(0);
			Serial.println(total_distance2);
			Serial.println("");

			//reading1 = pot1.readRaw();
			//reading2 = pot2.readRaw();
			String str1 = String((Voltage.readRaw()*5.0)/4095.0);
			str1.toCharArray(msg, 35);
			client.publish("dashboard/voltage", msg);
			//Serial.printf("%d", reading);
			String str2 = String((Current.readRaw()*5.0)/(4095.0));
			str2.toCharArray(msg, 35);
			client.publish("dashboard/current", msg);

			String str3 = String(velocity);
			str3.toCharArray(msg, 35);
			client.publish("dashboard/speed", msg);

			String str4 = String(total_distance2)+",3";
			str4.toCharArray(msg, 35);
			client.publish("dashboard/tracking", msg);

			String str5 = "1";
			str5.toCharArray(msg, 35);
			client.publish("dashboard/acceleration", msg);

			start_time = millis();

		}
		if (millis() - start_time_lap > sample_period_lap)
		{
			lap=lap+1.0;
			String str6 = String(lap);
			str6.toCharArray(msg, 35);
			client.publish("dashboard/laps", msg);

			String str7 = String(lap*100.0/80.0);
			str7.toCharArray(msg, 35);
			client.publish("dashboard/progress", msg);

			start_time_lap = millis();
		}
		
	}
	client.loop();
}

//******************************************************************************************************************
//******* END MAIN *************************************************************************************************
//******************************************************************************************************************

//*****************************
//***    CONEXION WIFI      ***
//*****************************
void setup_wifi()
{
	delay(10);
	// Nos conectamos a nuestra red Wifi
	Serial.println();
	Serial.print("Conectando a ssid: ");
	Serial.println(ssid);

	WiFi.begin(ssid, password);

	while (WiFi.status() != WL_CONNECTED)
	{
		delay(500);
		Serial.print(".");
	}

	Serial.println("");
	Serial.println("Conectado a red WiFi!");
	Serial.println("Dirección IP: ");
	Serial.println(WiFi.localIP());
}

//*****************************
//***    CONEXION MQTT      ***
//*****************************

void reconnect()
{

	while (!client.connected())
	{
		Serial.print("Intentando conexión Mqtt...");
		// Creamos un cliente ID
		String clientId = "IOTICOS_H_W_";
		clientId += String(random(0xffff), HEX);
		// Intentamos conectar
		if (client.connect(clientId.c_str(), mqtt_user, mqtt_pass))
		{
			Serial.println("Conectado!");
			// Nos suscribimos
			/*if (client.subscribe(root_topic_subscribe))
			{
				Serial.println("Suscripcion ok");
			}
			else
			{
				Serial.println("fallo Suscripciión");
			}*/
		}
		else
		{
			Serial.print("falló :( con error -> ");
			Serial.print(client.state());
			Serial.println(" Intentamos de nuevo en 5 segundos");
			delay(5000);
		}
	}
}

//*****************************
//***       CALLBACK        ***
//*****************************

void callback(char *topic, byte *payload, unsigned int length)
{
	String incoming = "";
	Serial.print("Mensaje recibido desde -> ");
	Serial.print(topic);
	Serial.println("");
	for (int i = 0; i < length; i++)
	{
		incoming += (char)payload[i];
	}
	incoming.trim();
	Serial.println("Mensaje -> " + incoming);
}