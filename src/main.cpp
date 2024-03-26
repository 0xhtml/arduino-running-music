// Importiere die benötigten Bibliotheken.
#include <I2C.h>
#include <SD.h>
#include <TMRpcm.h>

// Definiere die Register des MPU6050 für die spätere Nutzung. Die Werte
// basieren auf dem Datenblatt des MPU6050. Die Register werden für die
// manuelle Nutzung des MPU6050 benötigt, weil keine Bibliothek für den
// MPU6050 den Anforderungen dieses Projektes entspricht.
#define MPU_SMPLRT_DIV_REG 0x19
#define MPU_CONFIG_REG 0x1A
#define MPU_ACCEL_CONFIG_REG 0x1C
#define MPU_FIFO_EN_REG 0x23
#define MPU_ACC_OUT_REG 0x3B
#define MPU_USER_CTRL_REG 0x6A
#define MPU_PWR_MGMT_REG_1 0x6B
#define MPU_PWR_MGMT_REG_2 0x6C
#define MPU_FIFO_COUNT_REG 0x72
#define MPU_FIFO_R_W_REG 0x74
#define MPU_WHOAMI_REG 0x75

// Definiere die I2C-Adressen der beiden MPU6050. Hier ist es egal welcher
// Sensor am linken und rechten Bein befestigt ist, daher werden die beiden
// Sensoren als MPU1 und MPU2 bezeichnet.
#define MPU1_ADR 0x68
#define MPU2_ADR 0x69

// Aufgrund von Performance-Anforderungen, die eine simple Implementation der
// Songauswahl voraussetzten, und gleichzeitiger Limitierung der RAM-Nutzung,
// muss die Anzahl der möglichen Songs auf 15 limitiert werden.
#define MAX_SONG_ANZAHL 15

// Initialisiere globale Variablen für:
// - den aktuell genutzten MPU6050 (mpuAdr)
// - den Zeitpunkt des letzten Schrittes (letzterSchritt)
// - die aktuelle berechnete Schrittfrequenz (schritteProMinute)
// - die BPM aller verfügbaren Songs auf der SD-Karte (songBPMs)
// - die Anzahl der auf der SD-Karte gefundenen Songs (songAnzahl)
// - die BPM des aktuell laufenden Songs (aktuelleSongBPM)
uint8_t mpuAdr = MPU2_ADR;
unsigned long letzterSchritt = 0;
float schritteProMinute = 110;
uint8_t songBPMs[MAX_SONG_ANZAHL];
uint8_t songAnzahl = 0;
uint8_t aktuelleSongBPM = 0;

// Initialisiere die TMRpcm-Bibliothek, die für das Abspielen der Songs
// verantwortlich ist.
TMRpcm tmrpcm;

void err(int code, const char *msg) {
    // Die err-Funktion wird benutzt, um das Ausgeben von Fehlermeldungen zu
    // erleichtern. Ist die Bedingung, die als code übergeben wurde, erfüllt
    // oder der Fehlercode, der als code übergeben wurde, nicht 0, wird die
    // Meldung msg ausgegeben und die Programmausführung durch eine
    // Endlosschleife gestoppt.
    if (code == 0) return;
    Serial.print(msg);
    Serial.print(" (Code ");
    Serial.print(code);
    Serial.println(")");
    while(1);
}

void readN(uint8_t adr, uint8_t reg, uint8_t n) {
    // Wrapper um die I2c.read-Funktion, um Werte zu empfangen, jedoch noch
    // nicht zurückzugeben und im Falle eines Fehler eine Fehlermeldung
    // auszugeben.
    err(I2c.read(adr, reg, n), "Fehler beim Lesen");
}

uint8_t read(uint8_t adr, uint8_t reg) {
    // Wrapper um die I2c.read- und .receive-Funktion, um im Falle eines
    // Fehler eine Fehlermeldung über die err-Funktion auszugeben.
    readN(adr, reg, 1);
    return I2c.receive();
}

void write(uint8_t adr, uint8_t reg, uint8_t wert) {
    // Wrapper um die I2c.write-Funktion, um im Falle eines Fehlers ebenfalls
    // eine Fehlermeldung durch die err-Funktion aus zu geben.
    err(I2c.write(adr, reg, wert), "Fehler beim Schreiben");
}

void initialisiereMPU6050(uint8_t addr) {
    // Prüfe durch Auslesen des WHOAMI-Registers, ob die Adresse zu einem
    // MPU6050 gehört. Ein MPU6050 sollte mit dem Wert 0x68 antworten. Wenn der
    // MPU6050 nicht mit 0x68 antwortet, wird durch die err-Funktion die
    // Fehlermeldung ausgegeben und die Programmausführung gestoppt.
    err(read(addr, MPU_WHOAMI_REG) != 0x68, "Kein MPU6050 verbunden");

    // Der MPU6050 wird durch Setzen des höchstwertigen Bit des PWR_MGMT_1-
    // Register zurückgesetzt. Anschließend wird gewartet, bis der MPU6050 das
    // Bit zurück auf 0 setzt. Aus Erfahrung muss noch weitere 100ms gewartet
    // werden, um sicherzustellen, dass der MPU6050 genügend Zeit hat, um sich
    // vollständig zurückzusetzen.
    write(addr, MPU_PWR_MGMT_REG_1, 0b10000000);
    while(read(addr, MPU_PWR_MGMT_REG_1) & 0b10000000) delay(1);
    delay(100);

    // Aktiviere den internen 8kHz-Oszillator des MPU6050 als Taktgeber für die
    // Messungen und aktiviere ausschließlich den Beschleunigungssensor. Durch
    // deaktivieren des Gyroskop und des Temperatursensors kann Strom gespart
    // werden und die Messungen des Beschleunigungssensors sind genauer.
    write(addr, MPU_PWR_MGMT_REG_1, 0b00001000);
    write(addr, MPU_PWR_MGMT_REG_2, 0b00000111);

    // Setze die Abtastrate geringer, damit der MPU6050 nicht mehr Daten
    // produziert, als der Arduino verarbeiten kann. Die Abtastrate wird durch
    // den "sample rate divider" kontrolliert. Mit einem Teiler von 20 ergibt
    // sich eine Abtastrate von ca. 48Hz.
    write(addr, MPU_SMPLRT_DIV_REG, 20);

    // Setze den Messbereich des Beschleunigungssensors auf ±16g, weil der
    // Aufprall, wenn ein Schritt gemacht wird, erfahrungsgemäß durch den
    // MPU6050 als über das 16-fache der Erdgravitation gemessen werden kann.
    write(addr, MPU_ACCEL_CONFIG_REG, 0b00011000);

    // Schreibe die Beschleunigungsdaten in die FIFO-Warteschlange (first in,
    // first out), um Datenverlust zu vermeiden. Die FIFO-Schlange ermöglicht
    // es, Beschleunigungsdaten zu speichern, auch wenn der Arduino sie nicht
    // schnell genug auslesen kann. Dadurch wird verhindert, dass der Arduino
    // Beschleunigungsdaten, die zu einem Schritt gehören, überspringt.
    write(addr, MPU_USER_CTRL_REG, 0b01000000);
    write(addr, MPU_FIFO_EN_REG, 0b00001000);
}

float erkenneSchritteProMinute() {
    // Lese Anzahl der Daten in der FIFO-Warteschlange des MPU6050 ein. Weil
    // die Anzahl der Daten bis zu einem Wert größer des Maximalwertes, der in
    // einer einzelnen 8-Bit-Zahl gespeichert werden kann, steigen kann, ist
    // die Anzahl als eine 16-Bit-Zahl verteilt über zwei 8-Bit-Zahlen
    // repräsentiert. Deshalb werden zwei Werte vom MPU6050 gelesen und dann zu
    // einer 16-Bit-Zahl zusammengesetzt.
    readN(mpuAdr, MPU_FIFO_COUNT_REG, 2);
    uint16_t anzahl = I2c.receive() << 8 | I2c.receive();

    // Es muss immer eine Anzahl von Werten ausgelesen werden, die ein
    // vielfaches von 6 ist. Dies liegt daran, dass immer die Beschleunigungs-
    // werte von 3 Achsen, die sich je aus 2 Werten zusammensetzten, ausgelesen
    // werden. Sind weniger als 6 Werte bereit zum Auslesen ist der MPU6050
    // noch nicht vollständig fertig mit dem Schreiben der Werte. Wir ziehen
    // also von der anzahl den Rest nach dem Teilen durch 6 ab.
    anzahl -= anzahl % 6;

    // Wenn keine neuen Daten in der FIFO-Warteschlange sind, wurde auch kein
    // Schritt gemacht bzw. kann kein Schritt erkannt werden und wir können
    // bereits -1 für keinen Schritt zurückgeben.
    if (anzahl == 0) return -1;

    // Prüfe, dass nicht zu viele Daten in der FIFO-Warteschlange sind.
    // Theoretisch können bis zu 512 Werte in der Warteschlange gespeichert
    // werden. Jedoch schafft der Arduino es erfahrungsgemäß nicht mehr als
    // 32 Werte auf einmal auszulesen. Sollten mehr Werte vorhanden sein, kommt
    // der Arduino nicht mehr hinterher und das Programm muss gestoppt werden.
    err(anzahl > 32, "Zu viele Daten in der FIFO-Warteschlange");

    // Empfange `anzahl` viele Werte aus der FIFO-Warteschlange.
    readN(mpuAdr, MPU_FIFO_R_W_REG, anzahl);

    // Führe dies so lange aus, bis keine Daten mehr aus der Warteschlange
    // über sind.
    while (anzahl > 0) {
        // Berechne die Summe der quadrierten Beschleunigung der drei Achsen.
        // Auch die Beschleunigungswerte sind eine 16-Bit-Zahl, die als zwei
        // 8-Bit-Zahlen repräsentiert wird, gespeichert und müssen daher
        // zusammengesetzt werden.
        int64_t beschleunigung = 0;
        for (int i = 0; i < 3; i++) {
            int16_t wert = I2c.receive() << 8 | I2c.receive();
            beschleunigung += (int32_t) wert * (int32_t) wert;
        }

        // Es wurden 6 Werte aus der Warteschlange gelesen.
        anzahl -= 6;

        // Es wurde ein Schritt erkannt!
        if (beschleunigung > (4.0 * 4.0 * 2048 * 2048)) {
            unsigned long zeit = millis();

            float neueSchritteProMinute = 60000.0 / (zeit - letzterSchritt);
            if (neueSchritteProMinute > 230) return -1;

            letzterSchritt = zeit;

            return neueSchritteProMinute;
        }
    }

    // Der Wert -1 steht dafür, dass kein Schritt erkannt wurde. Gebe diesen
    // Wert zurück.
    return -1;
}

void ladeSongs() {
    // Öffne das Wurzelverzeichnis der SD-Karte.
    File ordner = SD.open("/");

    // Iteriere über alle Dateien im Wurzelverzeichnis der SD-Karte und
    // speichere die Datei immer in der Variable datei, bis keine Datei mehr
    // verfügbar ist.
    File datei;
    while ((datei = ordner.openNextFile())) {
        // Wandle den Dateinamen der Datei in einen Integer um. Hierbei wird
        // die Endung '.wav' sowie jeglicher Text ignoriert und wir erhalten
        // somit die BPM des Songs. Die BPM wird dann als letztes Element in
        // der Array songBPMs gespeichert.
        songBPMs[songAnzahl] = atoi(datei.name());

        // Erhöhe die songAnzahl um den neu gefundenen Song.
        songAnzahl++;

        // Falls die songAnzahl größer der Maximalanzahl ist, wird das Programm
        // unterbrochen.
        err(songAnzahl > MAX_SONG_ANZAHL, "Zu viele Songs.");
    }
    ordner.close();

    // Sortiere die BPMs in songBPMs nach ihrer Größe aufsteigend.
    qsort(
        songBPMs, songAnzahl, sizeof(songBPMs[0]),
        [](const void *a, const void *b){ return *(uint8_t*)a - *(uint8_t*)b; }
    );

    // Gebe zur Kontrolle die gefundenen Song BPMs aus.
    for (int i = 0; i < songAnzahl; ++i) {
        Serial.println(songBPMs[i]);
    }
}

void spiele(int songBPM) {
    // Die spiele-Funktion spielt den Song mit der angegebenen BPM songBPM ab,
    // indem der Dateiname durch das Umwandeln der songBPM in einen String und
    // das Anhängen von '.wav' generiert wird und anschließend mit TMRpcm
    // abgespielt wird.
    char dateiname[16] = "";
    itoa(songBPM, dateiname, 10);
    strcat(dateiname, ".wav");
    tmrpcm.play(dateiname);

    aktuelleSongBPM = songBPM;

    Serial.print("Spiele: ");
    Serial.println(dateiname);
}

void waehleSong() {
    if (abs(aktuelleSongBPM - schritteProMinute) < 10) return;

    uint8_t neueSongBPM = aktuelleSongBPM;

    for (uint8_t i = 0; i < songAnzahl; ++i) {
        if (songBPMs[i] > schritteProMinute) {
            neueSongBPM = songBPMs[i];
            break;
        }
    }

    if (neueSongBPM == aktuelleSongBPM) return;

    spiele(neueSongBPM);
}

void setup() {
    // Initialisiere Serial für die Ausgabe von Debug-Informationen und
    // Fehlermeldungen.
    Serial.begin(115200);
    Serial.println("Starte...");

    // Setze die Pins des Lautsprechers auf 9 und implizit 10.
    tmrpcm.speakerPin = 9;

    // Initialisiere die SD-Karte und lade Songs.
    err(!SD.begin(5), "Keine SD-Karte");
    ladeSongs();

    // Initialisiere I²C. Setzte das Zeitlimit für die I²C-Kommunikation auf
    // 10ms, damit sich der Arduino bei Fehlern in der I²C-Kommunikation nicht
    // aufhängt, sondern nach 10ms einen Fehler ausgibt und setzte die I²C-
    // Bibliothek auf den High-Speed-Modus, damit der Arduino schneller die
    // Daten der Beschleunigungssensoren auslesen kann.
    I2c.begin();
    I2c.timeOut(10);
    I2c.setSpeed(true);

    // Initialisiere beide MPU6050.
    initialisiereMPU6050(MPU1_ADR);
    initialisiereMPU6050(MPU2_ADR);
}

void loop() {
    // Starte Song neu, falls der Song zu Ende ist.
    if (!tmrpcm.isPlaying() && aktuelleSongBPM != 0) {
        spiele(aktuelleSongBPM);
    }

    // Erkenne Schritt und zugehörige BPM.
    float neueSchritteProMinute = erkenneSchritteProMinute();

    // Wenn kein Schritt erkannt wurde, können wir durch return zur nächsten
    // Iteration des loop übergehen.
    if (neueSchritteProMinute == -1) return;

    // Wechsel vom Linken zum Rechten oder Rechten zum linken MPU6050 und
    // verwerfe die nicht benötigten Daten des MPU6050.
    if (mpuAdr == MPU1_ADR) mpuAdr = MPU2_ADR;
    else mpuAdr = MPU1_ADR;
    write(mpuAdr, MPU_USER_CTRL_REG, 0b01000100);

    // Ignoriere Schritte mit einer BPM kleiner als 20.
    if (neueSchritteProMinute < 20) return;

    // Berechne neue durchschnittliche BPM. Der Durchschnitt wird mit einer
    // Gewichtung von 91% für den alten Wert und 9% für den neuen Wert
    // gebildet.
    schritteProMinute = .09 * neueSchritteProMinute + .91 * schritteProMinute;

    // Gebe die aktuelle BPM zu Kontrollzwecken seriell aus.
    Serial.print("Schritte Pro Minute: ");
    Serial.println(schritteProMinute);

    // Wähle einen passenden Song.
    waehleSong();
}
