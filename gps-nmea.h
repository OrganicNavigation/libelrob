#ifndef LIBGPS_NMEA_MESSAGES_H
#define LIBGPS_NMEA_MESSAGES_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
/*#include "Etime.h"*/
#include "libgps_struct.h"

#define GGA_STR_LEN 75  /* including margins */
#define RMC_STR_LEN 75  /* including margins */
#define ORIENT_VALID(chr) (chr == 'N' || chr == 'S' || chr == 'E' || chr == 'W')
#define LOG_SEP '\t'
#define GPS_NMEA_START_CHAR '$'


GPS_gga *GPS_GetGgaPtr();
GPS_rmc *GPS_GetRmcPtr();
void GPS_gps_alloc();
void GPS_PrintGGA();
void GPS_PrintRMC();
void PrintRMC(GPS_rmc *rmc);
void GPS_ClearGPSid();
int GPS_gps_parse_gga(char * line, TIMEVAL stamp);
int GPS_gps_parse_rmc(char * line, TIMEVAL stamp);
TIMEVAL GPS_GetMeasStampGGA();
TIMEVAL GPS_GetMeasStampRMC();
TIMEVAL GPS_GetAvailStampGGA();
TIMEVAL GPS_GetAvailStampRMC();
void GPS_SetMeasTimeStampGGA(TIMEVAL stamp);
void GPS_SetMeasTimeStampRMC(TIMEVAL stamp);
int  GPS_GetIDRMC();
TIMEVAL GPS_GetUTCGGA();
TIMEVAL GPS_GetUTCRMC();
void GPS_InitVoidGGA(GPS_gga *strct);
void GPS_InitVoidRMC(GPS_rmc *strct);
void GPS_InitVoidGSA(GPS_gsa *strct);
void GPS_InitVoidGST(GPS_gst *strct);

/** Read a GGA record from input */
int GPS_ReadGGAFromFile(FILE *input, GPS_gga *strct);

/** Read a RMC record from input */
int GPS_ReadRMCFromFile(FILE *input, GPS_rmc *strct);

/** Read a GSA record from input */
int GPS_ReadGSAFromFile(FILE *input, GPS_gsa *strct);

/** Read a GST record from input */
int GPS_ReadGSTFromFile(FILE *input, GPS_gst *strct);

/** Dump a header for the GPS logfile */
void GPS_DumpGPSHeader(FILE *strm, char *hostname);

/** Dump the current GGA struct */
void DMU_DumpGGA(FILE *strm);

/** Dump the current RMC struct */
void DMU_DumpRMC(FILE *strm);

void GPS_DumpRMC(FILE *strm, GPS_rmc *rmc);
void GPS_DumpGGA(FILE *strm, GPS_gga *gga);
void GPS_DumpGSA(FILE *strm, GPS_gsa *gsa);
void GPS_DumpGST(FILE *strm, GPS_gst *gst);

int GPS_Parse_GGA(GPS_gga *gga, char *line, TIMEVAL stamp);
int GPS_Parse_RMC(GPS_rmc *rmc, char *line, TIMEVAL stamp);
int GPS_Parse_GSA(GPS_gsa *gsa, char *line, TIMEVAL stamp);
int GPS_Parse_GST(GPS_gst *gst, char *line, TIMEVAL stamp);
void GPS_PrintGSA(GPS_gsa *gsa);

#ifdef __cplusplus
}
#endif

#endif
