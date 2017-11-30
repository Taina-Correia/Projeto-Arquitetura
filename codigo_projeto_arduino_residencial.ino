// Bibliotecas Auxiliares
#include <LiquidCrystal.h>     // Biblioteca para Display LCD
#include <DHT.h>               // Biblioteca para Sensor DHT11(Temperatura)

// Portas Analógicas
#define LDR    A0 //define o pino analógica que está o sensor LDR (luminosidade)
#define DHTPIN A5 //define o pino analógica que está o sensor DHT11 (temperatura)

// Portas Digitais
#define rele1      7     // define a porta que está o relé 1
#define rele2      10    // define a porta que está o relé 2
#define Sensor_PIR 13    // define a porta que está o sensor PIR (presença)

//declaração das variáveis de leitura do LDR (sensor de luz) e DHT11 (sensor de temperatura)
int   valorLDR;
int TempEntrada;
float valorDHT11_Temp;

//declaração de variável para ler sensor PIR
boolean valorPIR;
double leituraSensor;


// Cria objeto dht da classe DHT
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// Cria objeto lcd da classe LiquidCrystal
LiquidCrystal lcd(12,  // RS
                  11,  // Enable (E)
                  5,  // DB4
                  4,  // DB5
                  3,  // DB6
                  2); // DB7
                  
void setup() {

  Serial.begin(9600);
  lcd.begin(16, 2);
  dht.begin();

  pinMode(rele1, OUTPUT);
  pinMode(rele2, OUTPUT);
  pinMode(Sensor_PIR, INPUT);

  lcd.clear();

}


void loop() {

  TempEntrada = analogRead(DHTPIN); // Faz leitura da porta A5 e já faz conversão de dados para temperaura
  valorDHT11_Temp = (500*TempEntrada)/1023;
  valorLDR = analogRead(LDR);                           // Faz leitura da porta A0 (LDR)
  valorLDR = map(valorLDR, 0, 1023, 0, 100);           // Faz mapeamento de dados equivalentes a 0 - 100 % de luminosidade

  // fazendo leitura do DHT11 a cada 2 segundos
  if (millis() - leituraSensor >= 2000 )
  { 
    valorDHT11_Temp = dht.readTemperature(); 

    // Imprime na serial os 2 sensores a cada 2s.
    Serial.print("Luminosidade(LDR): ");  Serial.print(valorLDR);        Serial.println("%");
    Serial.print("Temperatura(DHT11): "); Serial.print(valorDHT11_Temp); Serial.println("°C");
   

    leituraSensor = millis();
  }

  // Se luminosidade atingir 50%, ligará o relé1
  if (valorLDR > 50) {             
   digitalWrite(rele1, HIGH);   
  }   
      else   digitalWrite(rele1, LOW);
  
   // Se detectar movimento, liga relé2 se não, deixa apagado
   if(valorPIR) {
    digitalWrite(rele2, HIGH);   
   }   
      else digitalWrite(rele2, LOW);
   
// exibe a porcentagem de luz no LCD
  lcd.setCursor(0, 0);
  lcd.print("Luz: ");
  lcd.print(valorLDR);
  lcd.println("%         ");

// exibe a temperatura no LCD
  lcd.setCursor(0, 1);
  lcd.print("Temp: ");
  lcd.print(valorDHT11_Temp);
  lcd.write(B11011111); //Simbolo de graus celsius
  lcd.println("C        ");

}
