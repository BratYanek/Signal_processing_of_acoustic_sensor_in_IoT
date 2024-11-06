#include <driver/i2s.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WiFi.h>
#include <SPIFFS.h>
#include <arduinoFFT.h>

// Zdefiniowanie pinow do podlaczenia INMP441 pod port I2S 
#define I2S_WS 25
#define I2S_SD 33
#define I2S_SCK 32
#define LED 27
 
// Uzycie I2S Procesora 0
#define I2S_PORT I2S_NUM_0
 
// Definicja dlugosci buffora, czestotliwosci probkowania
#define bufferLen 1024 // do generowania przebiegu akustycznego lepiej 64
const double samplingFrequency = 44100; //96000
double vReal[bufferLen];
double vImag[bufferLen];
int16_t sBuffer[bufferLen];
//wywołanie funkcji arduinoFFT() z biblioteki
arduinoFFT FFT = arduinoFFT();
//---------------------------------Konfiguracja polaczenia wifi i przypisanie portu do serwera-----------------
const char* ssid = "Device";
const char* password = "password";

AsyncWebServer server(80);

//-------------------------------Konfiguracja magistrali i2s---------------------------------------------------- 
void i2s_install() {
  // Ustawienie parametrów procesora I2S 
  const i2s_config_t i2s_config = {
    .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = 96000,
    .bits_per_sample = i2s_bits_per_sample_t(16),
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S),
    .intr_alloc_flags = 0,
    .dma_buf_count = 8,
    .dma_buf_len = bufferLen,
    .use_apll = false
  };
 
  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
}
 
void i2s_setpin() {
  // konfiguracja pinow magistrali i2s
  const i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = -1,
    .data_in_num = I2S_SD
  };

  i2s_set_pin(I2S_PORT, &pin_config);
}
//------------------String pomiar to funkcja gdzie generowany jest przebieg fali akustycznej i wyswietlany w SPL(dB)------
String pomiar(){
  // Pobranie danych z magistrali i2s i zapisanie jej w bufforze
  size_t bytesIn = 0;
  esp_err_t result = i2s_read(I2S_PORT, &sBuffer, bufferLen, &bytesIn, portMAX_DELAY);
  
  if (result == ESP_OK)
  {    
    // Odczytanie danych z bufora
    int16_t samples_read = bytesIn /8 ; // usrednienie danych surowych przez 8 probek
      if (samples_read > 0) {
        float mean = 0;
        for (int16_t i = 0; i < (samples_read); ++i) { //odczytywanie danych z bufora
          mean += (sBuffer[i]);     
        }
        mean /= (samples_read); //usrednianie danych przez wartosc otrzymanych probek w celu pozbycia sie nadmiernych fluktuacji wykresu

        //otrzymanie danych RMS
        float srednia =sqrt( abs(mean) / (samples_read));

          //przeliczenie danych cyfrowych na dB SPL
          float dB = 20*log10(srednia/(2^(23) - 1)) - 26  + 94 + 3.1623+10; // 3.1623 to domyślny offset (sinusa RMS w stosunku do dBFS) można zmieniać w celu liniowej kalibracji
          //wyswietlenie przebiegu w serial plotterze i zwrocenie wartosci danych przebiegu akustycznego do serwera (musi byc string bo http tak wymaga
          Serial.println(dB); 
          return String(dB);
    }
  }
}

//----------String fft to funkcja zwracajaca spektrum otrzymanego przebiegu akustycznego.
String fft(){

  // Pobranie danych z magistrali i2s i zapisanie jej w bufforze
  size_t bytesIn = 0;
  esp_err_t result = i2s_read(I2S_PORT, &sBuffer, bufferLen, &bytesIn, portMAX_DELAY);
  
  if (result == ESP_OK)
  {    
    // Read I2S data buffer
    int16_t samples_read = bytesIn ; // odczytanie danych surowych 
      if (samples_read > 0) {
        for (int16_t i = 0; i < (samples_read); ++i) {
          vReal[i] = (sBuffer[i]); //odczytanie rzeczywistych wartosci sygnalu
          vImag[i] = 0.0;
        }

        //Wyslanie danych do biblioteki arduinoFFT gdzie obliczane jest spektrum sygnalu algorymem FFT
        FFT.Windowing(vReal, bufferLen, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
        FFT.Compute(vReal, vImag, bufferLen, FFT_FORWARD);
        FFT.ComplexToMagnitude(vReal, vImag, bufferLen);

        int d[512] = {0}; //wyczyszczenie tablicy
 
        for (int xi = 85; xi < (bufferLen/2); xi++) { //bufferLen/2 żeby odczytywać z całej częstotliwości (24kHz) 
           if ((vReal[xi]) > 20){ //filtr usuwajacy szum wynikajacy z zasilania plytki
            (d[xi]) = (vReal[xi]); //odczytanie danych i umieszczenie ich w tablicy
           }
           else {
            (d[xi]) = 0; //filtra usuwajacego szum wynikajacy z zasilania plytki
           }
        } //zapisanie tablicy w formie stringa aby przeslac na serwer http
        String data = "[";
        for (int i = 0; i < 256; i++) {
          data += String(d[i]);
          if (i < 255) data += ",";
        }
        data += "]";
        //Serial.println(data);
        return data; //zwrocenie danych do wyswietlenia spektrum sygnalu
    }
  }
}

void setup() {

  // Inicjalizacjia portu szeregowego
  Serial.begin(115200);
  Serial.println(" ");
 
  delay(1000);

  // instalacja zadanych parametrow magistrali i2s
  i2s_install();
  i2s_setpin();
  i2s_start(I2S_PORT);
 
 
  delay(500);

  pinMode(LED, OUTPUT); //ustawienie mozliwosci dzialania na diodzie
  digitalWrite(LED, LOW);
// Iicjalizacja SPIFFS (mozliwosci wczytywania pliku z rozszerzeniem http do pamieci mikrokontrolera
  if(!SPIFFS.begin()){
    Serial.println("wystapil blad uruchamiajac SPIFFS");
    return;
  }

// Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Nawiazywanie polaczenia WiFi..");
  }

  // Wypisanie w porcie szeregowym lokalnego adresu IP
  Serial.println(WiFi.localIP());

//------------------Komunikacja esp32 z serwerem HTTP (pierwszy server on to wyslanie prosby z danymi do SPIFFS
//------------------kolejne dwa server on toprzeslanie do serwera danych kolejno z funkcji pomiar() i fft()  
server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/index.html");
  });
server.on("/cisnienie", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", pomiar().c_str());
  });
server.on("/data", HTTP_GET, [](AsyncWebServerRequest *request){
   String response = fft();
   request->send(200, "text/plain", response);
});
//----------Uruchomienie serwera-----------------
server.begin();
}

//-----------inicjalizacja funkcji pomiar() i fft() w funkcji void loop() aby wykonywaly sie nonstop w petli
void loop() {
pomiar();
fft();
}
