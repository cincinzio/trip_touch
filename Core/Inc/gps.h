#ifndef INC_GPS_H_
#define INC_GPS_H_

#include "main.h"

// Definizione Header UBX
#define UBX_SYNC1 0xB5
#define UBX_SYNC2 0x62
#define GPS_DMA_BUF_SIZE 512

#pragma pack(push, 1) // Allineamento byte singolo per i dati binari

typedef struct __attribute__((packed)) {
    uint32_t iTOW;      // 0: GPS Time of Week
    uint16_t year;      // 4: Year (UTC)
    uint8_t  month;     // 6: Month
    uint8_t  day;       // 7: Day
    uint8_t  hour;      // 8: Hour
    uint8_t  min;       // 9: Minute
    uint8_t  sec;       // 10: Second
    uint8_t  valid;     // 11: Validity Flags
    uint32_t tAcc;      // 12: Time Accuracy
    int32_t  nano;      // 16: Nanoseconds
    uint8_t  fixType;   // 20: Fix Type
    uint8_t  flags;     // 21: Flags
    uint8_t  flags2;    // 22: Flags2
    uint8_t  numSV;     // 23: Num Satellites
    int32_t  lon;       // 24: Longitude
    int32_t  lat;       // 28: Latitude
    int32_t  height;    // 32: Height above ellipsoid
    int32_t  hMSL;      // 36: Height above mean sea level
    uint32_t hAcc;      // 40: Horizontal Accuracy
    uint32_t vAcc;      // 44: Vertical Accuracy
    int32_t  velN;      // 48: NED North velocity
    int32_t  velE;      // 52: NED East velocity
    int32_t  velD;      // 56: NED Down velocity
    int32_t  gSpeed;    // 60: Ground Speed (2-D)
    int32_t  headMot;   // 64: Heading of motion
    uint32_t sAcc;      // 68: Speed Accuracy
    uint32_t headAcc;   // 72: Heading Accuracy
    uint16_t pDOP;      // 76: Position DOP
    uint8_t  reserved1[6]; // 78: Riservati (byte 78-83)
    int32_t  headVeh;   // 84: Heading of vehicle
    int16_t  magDec;    // 88: Magnetic declination
    uint16_t magAcc;    // 90: Magnetic accuracy
} UBX_NAV_PVT_t; // Totale 92 bytes

typedef struct __attribute__((packed)){
    uint8_t version; uint8_t res1[3];
    uint32_t iTOW;
    uint32_t distance;
    uint32_t totalDistance;
    uint32_t distanceStd;
} UBX_NAV_ODO_t;


// Struttura NAV-VELNED (Class 0x01, ID 0x12)
typedef struct __attribute__((packed)) {
    uint32_t iTOW;       // ms - GPS Time of Week
    int32_t  velN;       // cm/s - Velocità verso Nord
    int32_t  velE;       // cm/s - Velocità verso Est
    int32_t  velD;       // cm/s - Velocità verso il basso
    uint32_t speed;      // cm/s - Velocità 3D
    uint32_t gSpeed;     // cm/s - Velocità al suolo (2D)
    int32_t  heading;    // deg * 1e-5 - Direzione del movimento
    uint32_t sAcc;       // cm/s - Accuratezza velocità
    uint32_t cAcc;       // deg * 1e-5 - Accuratezza direzione
} UBX_NAV_VELNED_t;


typedef enum {
    NAV_STATUS_OK,
    NAV_STATUS_TUNNEL
} NavStatus_t;

extern NavStatus_t current_nav_status;
extern uint32_t odo_at_tunnel_entry;  // Valore NAV-ODO all'entrata del tunnel
extern double   odo_offset;           // Correzione tra odometro interno e utente
extern double User_Odometer_m;



extern float    GPS_VelNord_mps;
extern float    GPS_VelEst_mps;
extern float    GPS_GroundSpeed_mps;
extern uint32_t GPS_VelAcc_cm_s;
extern float    GPS_Lat;
extern float    GPS_Lon;
extern uint8_t  GPS_Satelliti;
extern uint32_t GPS_Distance;




#pragma pack(pop)

// Variabili globali (dichiarate come extern per gli altri file)
extern float GPS_Lat, GPS_Lon;
extern uint32_t GPS_Distance;
extern uint8_t GPS_Satelliti;

// Prototipo della funzione
void GPS_ProcessData(uint16_t Size);
void GPS_Super_Odometer();
void Debug_Print();
void GPS_Print_Report(UBX_NAV_PVT_t *pvt, UBX_NAV_VELNED_t *vel, UBX_NAV_ODO_t *odo, uint8_t is_stationary);
void GPS_Reset_All_Odometers();

#endif /* INC_GPS_H_ */
