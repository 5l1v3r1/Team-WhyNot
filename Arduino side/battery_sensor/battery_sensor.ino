int analogInput = 1;
int refresh = 1000;
float vout = 0.0; // arduino'ya giren voltaj değeri
float vin = 0.0; // bataryamızın voltaj değeri
float R1 = 100000; // R1 direnci 100k
float R2 = 10000; // R2 direnci 10k
int value = 0; // analog port okunan değer (0, 1024)

// vout = vin*R2 / (R1+R2);

void setup() {
  // put your setup code here, to run once:
  pinMode(analogInput, INPUT);
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
  value = analogRead(analogInput);
  Serial.print("Value=");  
  Serial.println(value);

  if(value >= 1023)
  {
    Serial.print("Error");
    delay(refresh);
    return;
  }
  else if(value <= 0
  {
    Serial.print("Voltage = 0V");
    delay(refresh);
    return;
  }

  vout = (value*5)/1024;
  vin = vout*(R1+R2) / R2;

  Serial.print("Vout = ");
  Serial.print(vout);
  Serial.println("V");

  Serial.print("Vin = ");
  Serial.print(vin);
  Serial.println("V");
  
  delay(refresh);
}
