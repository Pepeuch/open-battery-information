#include <Arduino.h>
#include "OneWireESP32.h" // Utilisez la bibliothèque compatible ESP32
//#include <OneWireESP32.h>
//#include <Arduino.h>

/** Major version number (X.x.x) */
#define ARDUINO_OBI_VERSION_MAJOR 0
/** Minor version number (x.X.x) */
#define ARDUINO_OBI_VERSION_MINOR 2
/** Patch version number (x.x.X) */
#define ARDUINO_OBI_VERSION_PATCH 1

// Configurez la broche OneWire. Ici, GPIO12 est utilisé comme exemple.
OneWire32 makita(12);

// Fonction pour envoyer une commande et lire avec la commande 0x33
void cmd_and_read_33(byte *cmd, uint8_t cmd_len, byte *rsp, uint8_t rsp_len) {
    int i;
    makita.reset();
    delayMicroseconds(400);
    makita.write(0x33, 0);

    uint8_t data = 0;
    for (i = 0; i < 8; i++) {
        delayMicroseconds(90);
        if (makita.read(data)) {
            rsp[i] = data;
        } else {
            rsp[i] = 0; // Valeur par défaut en cas d'erreur
        }
    }

    for (i = 0; i < cmd_len; i++) {
        delayMicroseconds(90);
        makita.write(cmd[i], 0);
    }

    for (i = 8; i < rsp_len + 8; i++) {
        delayMicroseconds(90);
        if (makita.read(data)) {
            rsp[i] = data;
        } else {
            rsp[i] = 0; // Valeur par défaut en cas d'erreur
        }
    }
}

// Fonction pour envoyer une commande et lire avec la commande 0xCC
void cmd_and_read_cc(byte *cmd, uint8_t cmd_len, byte *rsp, uint8_t rsp_len) {
    int i;
    makita.reset();
    delayMicroseconds(400);
    makita.write(0xcc, 0);

    uint8_t data = 0;
    for (i = 0; i < cmd_len; i++) {
        delayMicroseconds(90);
        makita.write(cmd[i], 0);
    }

    for (i = 0; i < rsp_len; i++) {
        delayMicroseconds(90);
        if (makita.read(data)) {
            rsp[i] = data;
        } else {
            rsp[i] = 0; // Valeur par défaut en cas d'erreur
        }
    }
}

// Fonction pour envoyer une commande générique et lire les données
void cmd_and_read(byte *cmd, uint8_t cmd_len, byte *rsp, uint8_t rsp_len) {
    int i;
    makita.reset();
    delayMicroseconds(400);

    uint8_t data = 0;
    for (i = 0; i < cmd_len; i++) {
        delayMicroseconds(90);
        makita.write(cmd[i], 0);
    }

    for (i = 0; i < rsp_len; i++) {
        delayMicroseconds(90);
        if (makita.read(data)) {
            rsp[i] = data;
        } else {
            rsp[i] = 0; // Valeur par défaut en cas d'erreur
        }
    }
}

// Initialisation du système
void setup() {
    Serial.begin(9600); // Démarrage de la communication série

    // Configuration des broches pour ESP32
    pinMode(13, OUTPUT); // GPIO13 remplace la broche 8 (sortie RTS)
    pinMode(2, OUTPUT);  // GPIO2 remplace la broche 2
}

// Fonction pour envoyer les données via USB (série)
void send_usb(byte *rsp, byte rsp_len) {
    for (int i = 0; i < rsp_len; i++) {
        Serial.write(rsp[i]);
    }
}

// Fonction pour lire les commandes depuis USB (série)
void read_usb() {
    if (Serial.available() >= 4) {
        byte start = Serial.read();
        byte cmd;
        byte len;
        byte data[255];
        byte rsp[255];
        byte rsp_len;

        if (start == 0x01) {
            len = Serial.read();
            rsp_len = Serial.read();
            cmd = Serial.read();
            if (len > 0) {
                for (int i = 0; i < len; i++) {
                    while (Serial.available() < 1);
                    data[i] = Serial.read();
                }
            }
        } else {
            return;
        }

        // Active RTS
        digitalWrite(13, HIGH);
        delay(400);

        uint8_t data_read = 0;
        switch (cmd) {
            case 0x01:
                rsp[0] = 0x01;
                rsp[2] = ARDUINO_OBI_VERSION_MAJOR;
                rsp[3] = ARDUINO_OBI_VERSION_MINOR;
                rsp[4] = ARDUINO_OBI_VERSION_PATCH;
                break;
            case 0x31:
                makita.reset();
                delayMicroseconds(400);
                makita.write(0xcc, 0);
                delayMicroseconds(90);
                makita.write(0x99, 0);
                delay(400);
                makita.reset();
                delayMicroseconds(400);
                makita.write(0x31, 0);
                delayMicroseconds(90);
                if (makita.read(data_read)) {
                    rsp[3] = data_read;
                }
                delayMicroseconds(90);
                if (makita.read(data_read)) {
                    rsp[2] = data_read;
                }
                delayMicroseconds(90);
                break;
            case 0x32:
                makita.reset();
                delayMicroseconds(400);
                makita.write(0xcc, 0);
                delayMicroseconds(90);
                makita.write(0x99, 0);
                delay(400);
                makita.reset();
                delayMicroseconds(400);
                makita.write(0x32, 0);
                delayMicroseconds(90);
                if (makita.read(data_read)) {
                    rsp[3] = data_read;
                }
                delayMicroseconds(90);
                if (makita.read(data_read)) {
                    rsp[2] = data_read;
                }
                delayMicroseconds(90);
                break;
            case 0x33:
                cmd_and_read_33(data, len, &rsp[2], rsp_len);
                break;
            case 0xCC:
                cmd_and_read_cc(data, len, &rsp[2], rsp_len);
                break;
            default:
                rsp_len = 0;
                break;
        }
        rsp[0] = cmd;
        rsp[1] = rsp_len;
        send_usb(rsp, rsp_len + 2);

        // Désactive RTS
        digitalWrite(13, LOW);
    }
}

// Boucle principale
void loop() {
    read_usb();
}
