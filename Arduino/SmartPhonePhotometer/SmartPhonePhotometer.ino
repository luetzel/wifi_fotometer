/*--------------------------------------------------
Photometer Programm
ein LED Photometer mit dem Lichtsensor TSL2561
mit eigenem WIFI Access Point und kleinem Webserver
HAW Hamburg - Labor BPA - Ulrich Scheffler
Version 0.26            07.11.2016
Version 0.26.1          16.02.2019
Version 0.26.2          14.04.2019
--------------------------------------------------*/
#include <ESP8266WiFi.h>         // Bibliothek für die ESP8266 Baugruppe und seine WIFI Funktionalität
//#include <Wire.h>                // Bibliothek für die I2C Datenkommunikation - hier zum Datenaustausch mit dem Lichtsensor benötigt
#include <Adafruit_Sensor.h>     // Basis-Bibliothek für die Adafruit Sensor Bibliotheken (Bibliothek von Adafruit)
#include <Adafruit_TSL2561_U.h>  // Bibliothek für den Lichtsensor TSL2661 (Bibliothek von Adafruit)
String sVersion ="Version 0.26.2 vom 14.04.2019 - RHG Berlin, Dr. M. Luetzelberger";
const char* ssid = "Photometer-";  // der erste Teil des Namens des WIFI Access Points
const char* password = "";         // Auf "" (leerer String) eingestellt für einen offenen Access Point ohne Passwort
unsigned long ulReqcount; // Zähler für die Aufrufe der Webseite
WiFiServer server(80); /* Eine Instanz auf dem Server für Port 80 erzeugen */
// IO-Anschlüsse konfigurieren
const int ledPin = 12; // LED an GPIO/Pin 12 / D6   Anschluss-Pin für die LED
const int sdaPin =  4; // SDA an GPIO/Pin  4 / D2   Anschluss-Pin für das SDA-Signal zur Datenkommunikation mit dem Lichtsensor
const int sclPin =  5; // SCL an GPIO/Pin  5 / D1   Anschluss-Pin für das SCL-Signal zur Datenkommunikation mit dem Lichtsensor
const int data_runs  = 7;   // Anzahl der Wiederholungsmessungen aus denen dann ein Mittelwert gebildet wird
float VIS_IRZEROdata = 0.0; // Für den aktuellen Nullproben-Messwert vom Lichtsensor.
                            //        Hier der Messwert vom Sensorteil, der im VIS und IR Bereich misst
float IRZEROdata     = 0.0; // Für den aktuellen Nullproben-Messwert vom Lichtsensor.
                            //        Hier der Messwert vom Sensorteil, der nur im IR Bereich misst
float LUXZEROdata    = 0.0; // Für den aktuellen Nullproben-Messwert vom Lichtsensor.
                            //        Hier die laut Datenblatt berechnete Beleuchtungsstärke in Lux
float VIS_IRdata     = 0.0; // Für den aktuellen Proben-Messwert vom Lichtsensor.
                            //        Hier der Messwert vom Sensorteil, der im VIS und IR Bereich misst
float IRdata         = 0.0; // Für den aktuellen Proben-Messwert vom Lichtsensor.
                            //        Hier der Messwert vom Sensorteil, der nur im IR Bereich misst
float LUXdata        = 0.0; // Für den aktuellen Proben-Messwert vom Lichtsensor.
                            //        Hier die laut Datenblatt berechnete Beleuchtungsstärke in Lux
float E_LUX          = 0.0; // Für die berechnete Extinktion aus den Beleuchtungsstärke-Daten
float E_VIS_IR       = 0.0; // Für die berechnete Extinktion aus den Sensor-Daten vom Sensorteil der im VIS und IR Bereich misst
float E_IR           = 0.0; // Für die berechnete Extinktion aus den Sensor-Daten vom Sensorteil der nur im IR Bereich misst
int probenzeilenIndex     = 0;    // Zeiger auf die aktuell zu füllende Probenzeile
const int probenzeilenMax = 12;    // Anzahl der Zeilen für Proben in der HTML Tabelle
float LUX_werte[probenzeilenMax]; // Array für Proben-Messwerte(Beleuchtungsstärke-Daten) vom Lichtsensor in Lux
float E_werte[probenzeilenMax];   // Array für berechnete Extinktionen aus den Beleuchtungsstärke-Daten
uint16_t broadband = 0; // Sensor-Daten vom Sensorteil der im VIS und IR Bereich misst
uint16_t infrared  = 0; // Sensor-Daten vom Sensorteil der nur im IR Bereich misst
// I2C-Addresse 0x39 verwenden - ADR nicht beschaltet (TSL2561_ADDR_FLOAT)
Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);
/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
void displaySensorDetails(void) { /* Zeigt einige Basisinformationen über den Sensor an */
  sensor_t sensor;
  tsl.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" lux");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" lux");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" lux");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}
/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
void configureSensor(void) { // Verstärkung und Integrationszeit konfigurieren
  // tsl.setGain(TSL2561_GAIN_1X); // 1-fache Verstärkung-bei hoher Beleuchtungsstärke, um eine Sensor-Übersättigung zu verhindern
  tsl.setGain(TSL2561_GAIN_16X);   // 16-fache Verstärkung-bei niedriger Beleuchtungsstärke, um die Sensor-Empfindlichkeit zu erhöhen
  // tsl.enableAutoRange(true);    // Automatische Verstärkung - Verstärkung wechselt automatisch zwischen 1-fach und 16-fach
  // Das Ändern der Integrationszeit bewirkt eine Änderung der Sensor-Genauigkeit bzw. seiner Auflösung (402ms = 16-bit data)
  //tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);    /*  13ms: Schnell aber niedrige Auflösung */
  //tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS); /* 101ms: Mittlere Geschwindigkeit und Auflösung */
  tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS); /* 402ms: Langsamste Geschwindigkeit aber hohe Auflösung (16-bit Daten)*/
  /* Einstellungen zur Info Ausgeben */
  Serial.println("------------------------------------");
  Serial.print  ("Gain:         "); Serial.println("Auto");
  Serial.print  ("Timing:       "); Serial.println("13 ms");
  //Serial.print  ("Timing:       "); Serial.println("101 ms");
  Serial.println("------------------------------------");
}
/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
bool readSensor() { // Sensor-Messdaten auslesen
  sensors_event_t event; // Ein neues TSL2561 Lichtsensor Ereignis einrichten
  int ok = 0;                 // Zähler für geglückte Messungen
  float LUX[data_runs+1];     // Daten Array zur Aufnahme der Einzelmessungen
  float VIS_IR[data_runs+1];  // Daten Array zur Aufnahme der Einzelmessungen
  float IR[data_runs+1];      // Daten Array zur Aufnahme der Einzelmessungen
  float LUX_max = 0.0, LUX_min = 0.0;
  for (int j=0; j <= data_runs-1; j++){   // Daten Arrays mit "0.0"-Werten vorbelegen
    LUX[j]    = 0.0;
    VIS_IR[j] = 0.0;
    IR[j]     = 0.0;
  }
  digitalWrite(ledPin, HIGH);   // LED einschalten
  for (int j=0; j <= data_runs-1; j++){
    // Messungen "data_runs"-mal wiederholen und Einzelmessungen in Daten Array eintragen
    tsl.getEvent(&event);
    if (event.light) {  // Nur dann "Wahr" wenn erfolgreich gemessen wurde
      LUX[j]= (event.light*1.0);
      if (j==0) {
        LUX_max = LUX[j];
        LUX_min = LUX[j];
      }
      if (LUX[j] > LUX_max) LUX_max = LUX[j];
      if (LUX[j] < LUX_min) LUX_min = LUX[j];
      tsl.getLuminosity (&broadband, &infrared);
      VIS_IR[j] = (broadband*1.0);
      IR[j]     = (infrared*1.0);
      delay(50);  // 50ms zwischen den Messungen warten
      ok += 1;  // Zähler für geglückte Messungen um 1 erhöhen
    }
    else { /* Wenn "event.light = 0 lux" ist, dann ist der Sensor möglicher Weise auch gesättigt
           und es können keinen Daten generiert werden! */
      Serial.println("Der Sensor-Messwert ist unbrauchbar. -> Sensor Fehler!");
    }
  }
  digitalWrite(ledPin, LOW);  // LED ausschalten
  if (ok>=data_runs) {  // Nur wenn alle Einzelmessungen erfolgreich durchgeführt wurden ...
    for (int j=0; j <= data_runs-1; j++){  // Einzelmessungen aufaddieren
      LUX[data_runs] += LUX[j];
      VIS_IR[data_runs] += VIS_IR[j];
      IR[data_runs] += IR[j];
    }
    /* Die aufaddierte Summe der Einzelwerte ohne den Größten-Wert und den Kleinsten-Wert geteilt durch Anzahl der Einzelmessungen
       minus 2 ergibt den bereinigten Mittelwert */
    LUX[data_runs]    = (LUX[data_runs]-LUX_max-LUX_min)    / (data_runs*1.0-2.0);
    // Die aufaddierte Summe der Einzelwerte geteilt durch Anzahl der Einzelmessungen ergibt den Mittelwert
    VIS_IR[data_runs] = VIS_IR[data_runs] / (data_runs*1.0);
    IR[data_runs]     = IR[data_runs]     / (data_runs*1.0);
    LUXdata    = LUX[data_runs];
    VIS_IRdata = VIS_IR[data_runs];
    IRdata     = IR[data_runs];
    Serial.print("" );Serial.print(LUX[data_runs]); Serial.print(" lx");
    Serial.print(" max: "); Serial.print(LUX_max); Serial.print(" lx");
    Serial.print(" min: "); Serial.print(LUX_min); Serial.print(" lx");
    Serial.print(" VIS_IR: "); Serial.print(VIS_IR[data_runs]);
    Serial.print(" IR: "); Serial.println(IR[data_runs]);
    return true;
  }
  else { Serial.println("Sensor Fehler!" ); return false; }
}
/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
void setup() {
  // Globale Voreinstellungen
  pinMode(ledPin, OUTPUT);    // Den Anschluss für die LED als Ausgang konfigurieren
  ulReqcount=0;               // Seitenaufrufzähler auf 0 setzten
  Serial.begin(9600);         // Serielle Verbindung initialisieren
  delay(500);                 // halbe Sekunde warten
  digitalWrite(ledPin, HIGH); // LED einschalten
  WiFi.mode(WIFI_AP);         // WIFI Access Point Modus initialisieren
  delay(5000);                // 5 Sekunden warten
  digitalWrite(ledPin, LOW);  // LED ausschalten
  for (int j=0; j < probenzeilenMax; j++) {  // Arrays mit "0.0"-Werten vorbelegen
    LUX_werte[j] = 0.0;
    E_werte[j]   = 0.0;
    Serial.print(j); Serial.print("-");
  }
  Serial.println("");
  // Das Anhängsel für den Access Point Namen (SSID) aus der MAC ID des ESP Moduls generieren.
  // Um den Namen nicht zu lang werden zu lassen, werden hier nur die letzten 2 Bytes verwendet.
  // Der Aufwand dient dazu, um einen möglichst einmaligen Access Point Namen für das jeweils
  // verwendete ESP Modul entstehen zu lassen.
  uint8_t mac[WL_MAC_ADDR_LENGTH];
  WiFi.softAPmacAddress(mac);
  String macID = String((mac[WL_MAC_ADDR_LENGTH - 2]*256 +  mac[WL_MAC_ADDR_LENGTH - 1]), DEC);
  Serial.println(macID);
  macID.toUpperCase();
  String AP_NameString = ssid + macID;
  Serial.print("MAC: "); Serial.println(AP_NameString);
  char AP_NameChar[AP_NameString.length() + 1];
  memset(AP_NameChar, 0, AP_NameString.length() + 1);
  for (unsigned int i=0; i<AP_NameString.length(); i++) AP_NameChar[i] = AP_NameString.charAt(i);
  // WIFI Access Point und den Web Server starten
  WiFi.softAP(AP_NameChar, password); // WiFi.softAP(ssid, password);
  server.begin();
  Serial.println("WEB-Server gestartet");
  Serial.print("WIFI Access Point gestartet. Namme (SSID) : "); Serial.println(AP_NameChar);
  Serial.print("Web Seite erreichbar unter der IP-Adresse): "); Serial.println(WiFi.softAPIP());
  Serial.println("");
  // Lichtsensor TSL2561
  //Wire.pins(sdaPin, sclPin);  // Wire.pins(int sda, int scl)
  Serial.println("Lichtsensor TSL2561"); Serial.println("");
  if(!tsl.begin()) { // Sensor initialisieren und im Fehlerfall eine Meldung ausgeben
    Serial.print("Ooops, es konnte kein TSL2561-Sensor erkannt werden ...! (Hardware Fehler)");
    while(1);
  }
  displaySensorDetails(); // Zeigt einige Basisinformationen über den Sensor an
  configureSensor();      // Verstärkung und Integrationszeit konfigurieren
  Serial.println("");
}
/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
void loop() {
  WiFiClient client = server.available(); // Prüfen ob ein WIFI-Client verbunden ist
  if (!client) return;  // Wenn keiner verbunden ist, wieder zum Anfang
  // Warten bis der WIFI-Client Daten gesendet hat
  Serial.println("Neuer WIFI-Client");
  unsigned long ultimeout = millis()+500;
  while(!client.available() && (millis()<ultimeout))delay(1);
  if(millis()>ultimeout) {
    Serial.println("WIFI-Client Fehler: Connection-Time-Out!");
    return;
  }
  // Die Erste Zeile der Anfrage lesen
  String sRequest = client.readStringUntil('\r');
  client.flush();
  // WIFI-Client stoppen, wenn die Anfrage leer ist
  if(sRequest == "") {
    Serial.println("WIFI-Client Fehler: Empty-Request! -> WIFI-Client angehalten");
    client.stop();
    return;
  }
  // "get path"; Das Ende des Pfads ist entweder ein " " oder ein "?"  (Syntax z.B. GET /?pin=SENSOR_A HTTP/1.1)
  String sPath="", sParam="", sCmd="";
  String sGetstart="GET ";
  int iStart, iEndSpace, iEndQuest;
  iStart = sRequest.indexOf(sGetstart);
  if (iStart >= 0) {
    iStart += +sGetstart.length();
    iEndSpace = sRequest.indexOf(" ",iStart);
    iEndQuest = sRequest.indexOf("?",iStart);
    // Den Pfad und die Parameter isolieren
    if(iEndSpace > 0) {
      if(iEndQuest > 0) {  // Pfad und Parameter
        sPath  = sRequest.substring(iStart,iEndQuest);
        sParam = sRequest.substring(iEndQuest,iEndSpace);
      }
      else { // Pfad aber keine Parameter
        sPath  = sRequest.substring(iStart,iEndSpace);
      }
    }
  }
  // Den Befehl isolieren
  if(sParam.length()>0) {
    int iEqu=sParam.indexOf("=");
    if(iEqu>=0) {
      sCmd = sParam.substring(iEqu+1,sParam.length());
      Serial.println(sCmd);
    }
  }
  /* Die HTML Seite erzeugen */
  String sResponse,sHeader;
  /* Für unpassenden Pfad eine 404-Fehlerseite generieren */
  if(sPath!="/") {
    sResponse = "<html><head><title>404 Not Found</title></head><body><h1>Not Found</h1>";
    sResponse += "<p>Die angeforderte Webseite (URL) gibt es auf diesem Server nicht.</p></body></html>";
    sHeader  = "HTTP/1.1 404 Not found\r\n";
    sHeader += "Content-Length: "; sHeader += sResponse.length(); sHeader += "\r\n";
    sHeader += "Content-Type: text/html\r\n"; sHeader += "Connection: close\r\n"; sHeader += "\r\n";
  }
  /* Für passenden Pfad ... */
  else {
    // Auf die Parameter reagieren ...
    if (sCmd.length()>0) {
      if(sCmd.indexOf("READZERO")>=0) {
        if(!readSensor()) {  // Messung durchführen und Sensordaten auslesen - ggf. Fehler melden
          Serial.println("Sensor overload");
        }
        else {
          LUXZEROdata    = LUXdata;
          VIS_IRZEROdata = VIS_IRdata;
          IRZEROdata     = IRdata;
        }
      }
      if(sCmd.indexOf("READTSL")>=0)  { // Wenn Button "Probe" gedrückt, ...
        if(!readSensor()) Serial.println("Sensor overload"); // Messung durchführen und Sensordaten auslesen - ggf. Fehler melden
        LUX_werte[probenzeilenIndex] =  LUXdata;
        // Die Extinktionen berechnen
        if ((LUXdata >0.0) & (LUXZEROdata > 0.0)) {
          E_LUX = -log10((LUXdata*1.0)/(LUXZEROdata*1.0));
        }
        if ((VIS_IRdata >0.0) & (VIS_IRZEROdata >0.0)) {
          E_VIS_IR = -log10((VIS_IRdata*1.0)/(VIS_IRZEROdata*1.0));
        }
        if ((IRdata >0.0) & (IRZEROdata > 0.0)) {
          E_IR = -log10((IRdata*1.0)/(IRZEROdata*1.0));
        }
        Serial.print("Index: ");   Serial.print(probenzeilenIndex);
        Serial.print(" - ");   Serial.print(LUX_werte[probenzeilenIndex],4);
        Serial.print(" E (LUX): ");   Serial.print(E_LUX,4);
        Serial.print(" (VIS_IR): "); Serial.print(E_VIS_IR,4);
        Serial.print(" (IR): ");     Serial.println(E_IR,4);
        probenzeilenIndex += 1;
        if (probenzeilenIndex >= probenzeilenMax) probenzeilenIndex = 0;
      }
    }
    // Die Extinktion mit ggf neuer Leerprobe erneut berechnen
    Serial.print("neu berechnet: E (LUX)");
    for (int i=0; i < probenzeilenMax; i++){
        if ((LUX_werte[i] >0.0) & (LUXZEROdata > 0.0)) {
          E_werte[i] = -log10((LUX_werte[i]*1.0)/(LUXZEROdata*1.0));
        }
         Serial.print(" : "); Serial.print(probenzeilenIndex-1); Serial.print(" - ");Serial.print(E_werte[probenzeilenIndex-1],4);
    }
    Serial.println(" ");
    ulReqcount++;  // Seitenaufrufzähler um 1 raufzählen
    sResponse  = "<html><head><title>VIS/IR-Photometer</title></head><body>";
    sResponse += "<font color=\"#000000\"><body bgcolor=\"#88B6E4\">";  // Hintergrundfarbe
    sResponse += "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=yes\">";
    sResponse += "<h1 style='text-align: center'>VIS/IR-Photometer</h1>";
    sResponse += "<p  style='text-align: center'><a href=\"?pin=READZERO\"><button style='font-size:26px'>Leerprobe</button></a>";
    sResponse += "&nbsp &nbsp &nbsp &nbsp        <a href=\"?pin=READTSL\"> <button style='font-size:26px'>Probe &nbsp";
    sResponse += probenzeilenIndex+1;
    sResponse += "</button></a></p>";
    sResponse += "<p style='text-align: center'>"; sResponse += "<FONT SIZE=+3>"; sResponse += LUXdata; sResponse += " lx";
    sResponse += "<FONT SIZE=-3>"; sResponse += "<BR>";
    sResponse += "<FONT SIZE=-1>"; sResponse += " VIS_IR: "; sResponse += VIS_IRdata; sResponse += "&nbsp &nbsp &nbsp &nbsp";
    sResponse += " IR: "; sResponse += IRdata; sResponse += "<BR>";
    sResponse += "E<sub>LUX</sub>: "; sResponse += String(E_LUX,4); sResponse += "&nbsp &nbsp";
    sResponse += "E<sub>VIS_IR</sub>: "; sResponse += String(E_VIS_IR,4); sResponse += "&nbsp &nbsp";
    sResponse += "E<sub>IR</sub>: "; sResponse += String(E_IR,4);
    sResponse += "<FONT SIZE=+1>"; sResponse += "</p>";
    sResponse += "<table border='1' width='100%'><tr><th> </th> <th>Messwert [lx]</th> <th>Extinktion [-]</th></tr>";
    Serial.println("           Messwert [lx]   Extinktion [-]");
    sResponse += "<tr><td>Leerprobe</td>";
    Serial.print  ("Leerprobe  ");
    sResponse +="<td style='text-align: center'>"; sResponse +=LUXZEROdata   ; sResponse +="</td>";
    Serial.println(LUXZEROdata);
    sResponse +="<td style='text-align: center'>"; sResponse +=" "; sResponse +="</td></tr>";
    for (int i=0; i < probenzeilenMax; i++){
      sResponse += "<tr><td>Probe "; sResponse += i+1; sResponse += "</td>";
      sResponse +="<td style='text-align: center'>"; sResponse +=String(LUX_werte[i],2) ; sResponse +="</td>";
      sResponse +="<td style='text-align: center'>"; sResponse +=String(E_werte[i],3)   ; sResponse +="</td></tr>";
      Serial.print  ("Probe  "); Serial.print  (i+1); Serial.print  ("  ");
      Serial.print  (String(LUX_werte[i],2)); Serial.print  ("  "); Serial.println(String(String(E_werte[i],3)));
    }
    sResponse += "</table>"; sResponse += "<BR>";
    sResponse += "<FONT SIZE=-2>"; sResponse += "Seitenaufrufe: "; sResponse += ulReqcount; sResponse += "<BR>";
    sResponse += sVersion; sResponse += "<BR>"; sResponse += "</body></html>";
    sHeader  = "HTTP/1.1 200 OK\r\n"; sHeader += "Content-Length: "; sHeader += sResponse.length(); sHeader += "\r\n";
    sHeader += "Content-Type: text/html\r\n"; sHeader += "Connection: close\r\n"; sHeader += "\r\n";
  }
  // Die Antwort an den WIFI-Client senden
  client.print(sHeader);  client.print(sResponse);
  // und den WIFI-Client stoppen
  client.stop(); Serial.println("WIFI-Client getrennt");
}
