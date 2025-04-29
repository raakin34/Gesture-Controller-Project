//----------------------------------------Load libraries
#include <esp_now.h>
#include <WiFi.h>
//----------------------------------------


#define IN1 5
#define IN2 18
#define IN3 19
#define IN4 21

#define ENA 2
#define ENB 4


#define IN5 26
#define IN6 25
#define IN7 33
#define IN8 32

#define ENC 14
#define END 27

#define LED_PIN1 22
#define LED_PIN2 23


//----------------------------------------Define variables to store incoming readings
int receive_value = 0;
//----------------------------------------

typedef struct struct_message {
    int value;
} struct_message;

struct_message receive_Data; //--> Create a struct_message to receive data.
//----------------------------------------

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ Callback when data is received
void OnDataRecv(const esp_now_recv_info* recv_info, const uint8_t* incomingData, int len){
  memcpy(&receive_Data, incomingData, sizeof(receive_Data));
  Serial.println();
  Serial.println("<<<<< Receive Data:");
  Serial.print("Bytes received: ");
  Serial.println(len);
  receive_value = receive_Data.value;
  Serial.println("Receive Data: ");
  Serial.println(receive_value);
  Serial.println("<<<<<");
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


void setup() {

  Serial.begin(115200);
  
  WiFi.mode(WIFI_STA); //--> Set device as a Wi-Fi Station

    // put your setup code here, to run once:
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(IN5, OUTPUT);
  pinMode(IN6, OUTPUT);
  pinMode(IN7, OUTPUT);
  pinMode(IN8, OUTPUT);
  pinMode(ENC, OUTPUT);
  pinMode(END, OUTPUT);

  pinMode(LED_PIN1, OUTPUT);
  pinMode(LED_PIN2, OUTPUT);


  //----------------------------------------Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  //----------------------------------------
   //--> Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);

}

void loop() {
  // put your main code here, to run repeatedly:

  int gesture = receive_value / 10;  // Removes last digit → 403 / 10 = 40
  int speed = receive_value % 10;    // Last digit → 3


 if (gesture == 0) { //Stop

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  digitalWrite(IN5, LOW);
  digitalWrite(IN6, LOW);

  digitalWrite(IN7, LOW);
  digitalWrite(IN8, LOW);
  }


  if (gesture == 1) {   //Forward

  analogWrite(ENA, 100+(speed*17));
  analogWrite(ENB, 100+(speed*17));
  analogWrite(ENC, 100+(speed*17));
  analogWrite(END, 100+(speed*17));

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  digitalWrite(IN5, HIGH);
  digitalWrite(IN6, LOW);

  digitalWrite(IN7, HIGH);
  digitalWrite(IN8, LOW);
  }

  if (gesture == 2) { //Reverse

  analogWrite(ENA, 100+(speed*17));
  analogWrite(ENB, 100+(speed*17));
  analogWrite(ENC, 100+(speed*17));
  analogWrite(END, 100+(speed*17));

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  digitalWrite(IN5, LOW);
  digitalWrite(IN6, HIGH);

  digitalWrite(IN7, LOW);
  digitalWrite(IN8, HIGH);
  }

  if (gesture == 3) { //Right

  analogWrite(ENA, 100+(speed*17));
  analogWrite(ENB, 100+(speed*17));
  analogWrite(ENC, 100+(speed*17));
  analogWrite(END, 100+(speed*17));

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  digitalWrite(IN5, LOW);
  digitalWrite(IN6, HIGH);

  digitalWrite(IN7, HIGH);
  digitalWrite(IN8, LOW);
  }

  if (gesture == 4) { //Left

  analogWrite(ENA, 100+(speed*17));
  analogWrite(ENB, 100+(speed*17));
  analogWrite(ENC, 100+(speed*17));
  analogWrite(END, 100+(speed*17));

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  digitalWrite(IN5, HIGH);
  digitalWrite(IN6, LOW);

  digitalWrite(IN7, LOW);
  digitalWrite(IN8, HIGH);
  }

  if (gesture == 5) { //Stationary 
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  digitalWrite(IN5, LOW);
  digitalWrite(IN6, LOW);

  digitalWrite(IN7, LOW);
  digitalWrite(IN8, LOW);
  }

  if (gesture == 6) { //Flash 
  digitalWrite(LED_PIN1, HIGH);
  digitalWrite(LED_PIN2, HIGH);
  delay(500);
  digitalWrite(LED_PIN1, LOW);
  digitalWrite(LED_PIN2, LOW);
  delay(500); 
  }

  if (gesture == 7) {
  analogWrite(ENA, 255);
  analogWrite(ENB, 255);
  analogWrite(ENC, 255);
  analogWrite(END, 255);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  digitalWrite(IN5, HIGH);
  digitalWrite(IN6, LOW);

  digitalWrite(IN7, LOW);
  digitalWrite(IN8, HIGH);

  delay(405);

  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  analogWrite(ENC, 0);
  analogWrite(END, 0);
  }

  if (gesture == 8) {
  analogWrite(ENA, 255);
  analogWrite(ENB, 255);
  analogWrite(ENC, 255);
  analogWrite(END, 255);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  digitalWrite(IN5, LOW);
  digitalWrite(IN6, HIGH);

  digitalWrite(IN7, HIGH);
  digitalWrite(IN8, LOW);

  delay(385);

  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  analogWrite(ENC, 0);
  analogWrite(END, 0);



  }

  if (gesture == 9) {

  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  analogWrite(ENC, 0);
  analogWrite(END, 0);

  digitalWrite(LED_PIN2, HIGH);
  delay(500);
 
  digitalWrite(LED_PIN2, LOW);
  delay(500); 
  }

  if (gesture == 10) {

  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  analogWrite(ENC, 0);
  analogWrite(END, 0);

  digitalWrite(LED_PIN1, HIGH);
 
  delay(500);
  digitalWrite(LED_PIN1, LOW);

  delay(500); 
  }





  delay(10);

}
