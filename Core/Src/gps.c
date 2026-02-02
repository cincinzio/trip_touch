#include "gps.h"
#include <string.h>
#include <math.h>
#include <stdio.h>


// Definiamo le variabili qui (senza extern)
float GPS_Lat = 0.0f;
float GPS_Lon = 0.0f;
uint8_t GPS_Satelliti = 0;
float    GPS_VelNord_mps = 0.0f;
float    GPS_VelEst_mps  = 0.0f;
float    GPS_GroundSpeed_mps = 0.0f;
uint32_t GPS_VelAcc_cm_s = 0;
uint8_t  GPS_Fix = 0;
uint32_t GPS_Distance = 0, GPS_Distance_Total = 0;
int32_t  GPS_Heading = 0;

NavStatus_t current_nav_status = NAV_STATUS_OK;
uint32_t    odo_at_tunnel_entry = 0;
double      odo_offset = 0.0;
double      User_Odometer_m = 0.0;

// Il buffer è definito nel main.c, lo richiamiamo qui
extern uint8_t gps_dma_buffer[GPS_DMA_BUF_SIZE];



/**
 * @brief Verifica la Checksum UBX
 * @param data: puntatore all'inizio del pacchetto (dopo l'header 0xB5 0x62)
 * @param len: lunghezza del payload + 4 (class, id, len_low, len_high)
 * @param ck_a: valore CK_A letto dal pacchetto
 * @param ck_b: valore CK_B letto dal pacchetto
 */
uint8_t UBX_VerifyChecksum(uint8_t* data, uint16_t len, uint8_t ck_a, uint8_t ck_b) {
    uint8_t a = 0, b = 0;

    for (uint16_t i = 0; i < len; i++) {
        a = a + data[i];
        b = b + a;
    }

    return (a == ck_a && b == ck_b);
}

// Helper per inviare stringhe via UART Debug
void Debug_Print(char *msg) {
    extern UART_HandleTypeDef huart3; // Assicurati che huart3 sia la tua porta debug
    HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 100);
}

void GPS_ProcessData(uint16_t Size) {

    // Usiamo variabili static per mantenere i dati tra una chiamata e l'altra
    static UBX_NAV_PVT_t    pvtData;
    static UBX_NAV_VELNED_t velData;
    static UBX_NAV_ODO_t    odoData;

    // Questa variabile tiene traccia di quali messaggi sono arrivati nel ciclo attuale
    // Bit 0: PVT, Bit 1: VELNED, Bit 2: ODO
    static uint8_t msg_received_mask = 0;

    for (uint16_t i = 0; i < Size - 8; ) {
        if (gps_dma_buffer[i] == UBX_SYNC1 && gps_dma_buffer[i+1] == UBX_SYNC2) {
            uint8_t  msgClass   = gps_dma_buffer[i+2];
            uint8_t  msgID      = gps_dma_buffer[i+3];
            uint16_t payloadLen = gps_dma_buffer[i+4] | (gps_dma_buffer[i+5] << 8);

            if (i + 6 + payloadLen + 2 > Size) break;

            if (UBX_VerifyChecksum(&gps_dma_buffer[i+2], 4 + payloadLen,
                gps_dma_buffer[i+6+payloadLen], gps_dma_buffer[i+7+payloadLen])) {

                uint8_t *payloadPtr = &gps_dma_buffer[i+6];

                if (msgClass == 0x01 && msgID == 0x07) { // NAV-PVT
                    memcpy(&pvtData, payloadPtr, sizeof(pvtData));
                    msg_received_mask |= (1 << 0); // Segna bit 0
                }
                else if (msgClass == 0x01 && msgID == 0x12) { // NAV-VELNED
                    memcpy(&velData, payloadPtr, sizeof(velData));
                    GPS_GroundSpeed_mps = velData.gSpeed / 100.0f;
                    msg_received_mask |= (1 << 1); // Segna bit 1
                }
                else if (msgClass == 0x01 && msgID == 0x09) { // NAV-ODO
                    memcpy(&odoData, payloadPtr, sizeof(odoData));
                    msg_received_mask |= (1 << 2); // Segna bit 2
                }

                i += (6 + payloadLen + 2);





                // --- IL CONTROLLO CRITICO ---
                // Se abbiamo ricevuto tutti e 3 i messaggi (1 | 2 | 4 = 7)
                if (msg_received_mask == 0x07) {
                    GPS_Super_Odometer(&pvtData, &velData, &odoData);
                    msg_received_mask = 0; // Resetta la checklist per il prossimo secondo
                }
                continue;
            }
        }
        i++;
    }
}

void GPS_Super_Odometer(UBX_NAV_PVT_t *pvt, UBX_NAV_VELNED_t *vel, UBX_NAV_ODO_t *odo) {
    static uint32_t last_itow = 0;
    static uint32_t stationary_start_tick = 0;
    static uint8_t  is_confirmed_stationary = 1;



    // 1. FILTRO QUALITÀ (Fix 3D necessario)
    if (pvt->fixType < 3) {
        // Logica tunnel qui...
        return;
    }

    // 2. CALCOLO DELTA TIME
    float dt = (last_itow != 0) ? (vel->iTOW - last_itow) / 1000.0f : 0;
    last_itow = vel->iTOW;
    if (dt <= 0 || dt > 1.5f) return;

    // 3. LOGICA DI CONGELAMENTO (Anti-Drift Hysteresis)
    float speed_mps = vel->gSpeed / 100.0f;

    // Soglia di movimento: 0.2 m/s (~0.7 km/h)
    // Soglia di precisione: sAcc < 50 cm/s
    if (speed_mps < 0.20f || vel->sAcc > 50) {
        if (!is_confirmed_stationary) {
            if (stationary_start_tick == 0) {
                stationary_start_tick = HAL_GetTick();
            } else if (HAL_GetTick() - stationary_start_tick > 2000) {
                // Dopo 2 secondi di velocità bassa, confermiamo lo stato FERMO
                is_confirmed_stationary = 1;
            }
        }
    } else {
        // Movimento rilevato: resettiamo il timer e usciamo dallo stato FERMO
        is_confirmed_stationary = 0;
        stationary_start_tick = 0;
    }

    // 4. ACCUMULO DISTANZA
    if (!is_confirmed_stationary) {
        User_Odometer_m += (double)(speed_mps * dt);
    } else {
        speed_mps = 0.0f; // Forza a 0 per il display se confermato fermo
    }

    // 5. CALIBRAZIONE CON NAV-ODO (Solo se in movimento)
    if (pvt->sec % 30 == 0 && !is_confirmed_stationary) {
        double diff = (double)odo->totalDistance - User_Odometer_m;
        if (fabs(diff) < 3.0) User_Odometer_m += diff * 0.1;
    }

    GPS_Print_Report(pvt, vel, odo, is_confirmed_stationary);
}

void GPS_Print_Report(UBX_NAV_PVT_t *pvt, UBX_NAV_VELNED_t *vel, UBX_NAV_ODO_t *odo, uint8_t is_stationary) {
    char report[300];

    // Calcoliamo la velocità in km/h per una lettura più naturale
    float speed_kmh = (vel->gSpeed / 100.0f) * 3.6f;

    // Formattazione Latitudine e Longitudine (usiamo double per la stampa)
    double lat = pvt->lat * 1e-7;
    double lon = pvt->lon * 1e-7;

    // Costruiamo la stringa completa
    sprintf(report,
        "\r\n--- GPS REPORT ---\r\n"
        "TIME: %02d:%02d:%02d | FIX: %d | SAT: %d\r\n"
        "POS:  %.7f, %.7f\r\n"
        "STATO: %s | VEL: %.2f km/h\r\n"
        "ODO CALC: %.2f m\r\n"
        "ODO INT:  %lu m\r\n"
        "------------------\r\n",
        pvt->hour, pvt->min, pvt->sec, pvt->fixType, pvt->numSV,
        lat, lon,
        is_stationary ? "FERMO (BLOCK)" : "IN MOVIMENTO",
        speed_kmh,
        User_Odometer_m,
        odo->totalDistance
    );

    Debug_Print(report);
}



void GPS_Reset_All_Odometers(void) {
    // 1. Reset variabile software locale
    User_Odometer_m = 0.0;

    // 2. Comando UBX-CFG-ODO (Reset Distanza)
    // Questo comando è standard per u-blox serie 7, 8 e 9
    uint8_t resetCmd[] = { 0xB5, 0x62, 0x01, 0x10, 0x00, 0x00, 0x11, 0x34 };;

    // Usiamo la huart6 che è quella corretta per il tuo sistema
    extern UART_HandleTypeDef huart6;

    HAL_StatusTypeDef status = HAL_UART_Transmit(&huart6, resetCmd, sizeof(resetCmd), 100);

    if (status == HAL_OK) {
        Debug_Print("\r\n[RESET] Comando inviato su UART6\r\n");
    } else {
        Debug_Print("\r\n[RESET] Errore hardware UART6\r\n");
    }
}
/* funzione ok ma conta anche da fermo

void GPS_ProcessData(uint16_t Size) {
    // Usiamo variabili static per mantenere i dati tra una chiamata e l'altra
    static UBX_NAV_PVT_t    pvtData;
    static UBX_NAV_VELNED_t velData;
    static UBX_NAV_ODO_t    odoData;

    // Questa variabile tiene traccia di quali messaggi sono arrivati nel ciclo attuale
    // Bit 0: PVT, Bit 1: VELNED, Bit 2: ODO
    static uint8_t msg_received_mask = 0;

    for (uint16_t i = 0; i < Size - 8; ) {
        if (gps_dma_buffer[i] == UBX_SYNC1 && gps_dma_buffer[i+1] == UBX_SYNC2) {
            uint8_t  msgClass   = gps_dma_buffer[i+2];
            uint8_t  msgID      = gps_dma_buffer[i+3];
            uint16_t payloadLen = gps_dma_buffer[i+4] | (gps_dma_buffer[i+5] << 8);

            if (i + 6 + payloadLen + 2 > Size) break;

            if (UBX_VerifyChecksum(&gps_dma_buffer[i+2], 4 + payloadLen,
                gps_dma_buffer[i+6+payloadLen], gps_dma_buffer[i+7+payloadLen])) {

                uint8_t *payloadPtr = &gps_dma_buffer[i+6];

                if (msgClass == 0x01 && msgID == 0x07) { // NAV-PVT
                    memcpy(&pvtData, payloadPtr, sizeof(pvtData));
                    msg_received_mask |= (1 << 0); // Segna bit 0
                }
                else if (msgClass == 0x01 && msgID == 0x12) { // NAV-VELNED
                    memcpy(&velData, payloadPtr, sizeof(velData));
                    GPS_GroundSpeed_mps = velData.gSpeed / 100.0f;
                    msg_received_mask |= (1 << 1); // Segna bit 1
                }
                else if (msgClass == 0x01 && msgID == 0x09) { // NAV-ODO
                    memcpy(&odoData, payloadPtr, sizeof(odoData));
                    msg_received_mask |= (1 << 2); // Segna bit 2
                }

                i += (6 + payloadLen + 2);

                // --- IL CONTROLLO CRITICO ---
                // Se abbiamo ricevuto tutti e 3 i messaggi (1 | 2 | 4 = 7)
                if (msg_received_mask == 0x07) {
                    GPS_Super_Odometer(&pvtData, &velData, &odoData);
                    msg_received_mask = 0; // Resetta la checklist per il prossimo secondo
                }
                continue;
            }
        }
        i++;
    }
} */
