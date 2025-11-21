#include <LCDWIKI_GUI.h> 
#include <LCDWIKI_KBV.h>
#include <DHT.h>

LCDWIKI_KBV mylcd(ILI9341,A3,A2,A1,A0,A4);

#define BLACK 0x0000
#define GREEN 0x07E0
#define RED 0xF800
#define BLUE 0x001F
#define WHITE 0xFFFF

#define DHT_PIN 15
#define DHT_TYPE DHT11

#define DELAY 2000

DHT dht(DHT_PIN, DHT_TYPE);

void setup() 
{
    Serial.begin(9600);
    mylcd.Init_LCD();
    Serial.println(mylcd.Read_ID(), HEX);
    mylcd.Fill_Screen(BLACK);
    mylcd.Set_Text_Mode(0);
    
    
    mylcd.Set_Rotation(0);
    uint8_t madctl = 0xA8; 
    mylcd.Push_Command(0x36, &madctl, 1);
    

    Serial.println("Initializing DHT...");
    dht.begin();
    delay(2000);
}

void loop(void) 
{   
    float humidity = dht.readHumidity();
    float temperature = dht.readTemperature();
    
    Serial.print("Humidity: ");
    Serial.print(humidity);
    Serial.print(" Temperature: ");
    Serial.println(temperature);
    
    if (isnan(humidity) || isnan(temperature)) {
        Serial.println("Failed to read from DHT sensor!");
        mylcd.Fill_Screen(BLACK);
        mylcd.Set_Text_Back_colour(BLACK);
        mylcd.Set_Text_colour(RED);
        mylcd.Set_Text_Size(2);
        mylcd.Print_String("DHT SENSOR ERROR", 50, 100);
        mylcd.Set_Text_Size(1);
        mylcd.Print_String("Check wiring/pull-up resistor", 30, 130);
    } else {
        mylcd.Fill_Screen(BLACK);
        mylcd.Set_Text_Back_colour(BLACK);
        
        mylcd.Set_Text_colour(WHITE);
        mylcd.Set_Text_Size(2);
        mylcd.Print_String("TEMPERATURE", 80, 30);
        
        mylcd.Set_Text_colour(GREEN);
        mylcd.Set_Text_Size(4);
        
        char tempStr[10];
        dtostrf(temperature, 5, 1, tempStr);
        mylcd.Print_String(tempStr, 100, 70);
        mylcd.Set_Text_Size(3);
        mylcd.Print_String("C", 220, 85);
        
        mylcd.Set_Text_colour(WHITE);
        mylcd.Set_Text_Size(2);
        mylcd.Print_String("HUMIDITY", 100, 140);
        
        mylcd.Set_Text_colour(BLUE);
        mylcd.Set_Text_Size(4);
        
        char humidStr[10];
        dtostrf(humidity, 5, 1, humidStr);
        mylcd.Print_String(humidStr, 100, 180);
        mylcd.Set_Text_Size(3);
        mylcd.Print_String("%", 220, 195);
    }
    
    delay(DELAY);
}
