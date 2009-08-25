#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <assert.h>

#include "Etime.h"
#include "Emacros.h"
#include "Edebug.h"
#include "gps-nmea.h"
#include "libgps.h"

const char GPS_GPS_FIX[7][64]={"fix not available","GPS fix","Differential GPS fix","","","","Estimated(DR) fix"};
const char GPS_AXIS_FMT[] = "%lf\t%lf\t%lf\t";
const char GPS_REAL_FMT[] = "%lf\t";
const char GPS_INT_FMT[]  = "%i\t";
const char GPS_CHAR_FMT[] = "%c\t";

static char DUMP_FMT_GGA[256]    = "#";
static char DUMP_FMT_RMC[256]    = "#";
static char DUMP_FMT_GSA[256]    = "#";
static char DUMP_FMT_GST[256]    = "#";
static GPS_gga *m_gpgga = NULL;
static GPS_rmc *m_gprmc = NULL;

/* Only use these two functions within the GPS thread function */
GPS_gga *GPS_GetGgaPtr()
{
  return m_gpgga;
}

GPS_rmc *GPS_GetRmcPtr()
{
  return m_gprmc;
}

void GPS_ClearGPSid()
{
  m_gpgga->meas_id = 0;
  m_gprmc->meas_id = 0;
}

/* return 1 if ok else 0*/
int GPS_CompChecksum(const char *line)
{
  size_t length;
  size_t i;
  unsigned char sum = 0;
  unsigned int true_chksum;
  char     *chksum;

  if((chksum = strchr(line, '*')) == NULL)
  {
    EERR("GPS_CompChecksum -> * not found in msg\n");
    return 0;
  }

  chksum += 1;
  length = (size_t) (chksum-line-2);

  sscanf(chksum,"%x",&true_chksum);

  for(i = 1; i <= length; i++)
    sum ^= line[i];

  /*  EDBG("Chksum = %X length = %i truth = %X\n",(int)sum, length, true_chksum);*/

  if(true_chksum != sum) {
    EERR("GPS_CompChecksum -> Wrong checksum\n");
    return 0;
  }
  return 1;
}

int GPS_Parse_GSA(GPS_gsa *gsa, char *line, TIMEVAL stamp)
{
  char *ptr;
  int  i;

  if(gsa == NULL) return 0;
  gsa->valid = 0;

  /* WARNING! By now we consider available time = measured time */
  /*Time_InitVoid(&gsa->meas_time);*/
  gsa->meas_time  = stamp;
  gsa->avail_time = stamp;
  gsa->meas_id += 1;

  if(strlen(line) < 36) return 0;

  if(!GPS_CompChecksum(line)) return 0;

  /* mode */
  ptr = strchr(line, ',');
  if (ptr == NULL) return 0;
  ptr++;
  gsa->mode = ptr[0];

  /* gps_quality */
  ptr = strchr(ptr, ',');
  if (ptr==NULL) return 0;
  ptr++;
  if(sscanf(ptr,"%i",&gsa->gps_quality)!=1) {
    EERR("Strange char received\n");
    return 0;
  }
  /*  gsa->gps_quality = atoi(ptr);*/

  /* satellites */
  for(i = 0; i < 12; i++) {
    ptr = strchr(ptr, ',');
    if(ptr == NULL) return 0;
    ptr += 1;
    /*gsa->PRN[i] = atoi(ptr);*/
    /*    EPRINT("sat: %i\n", gsa->PRN[i]);*/
  }

  /* pdop */
  ptr = strchr(ptr, ',');
  if (ptr == NULL) return 0;
  ptr++;
  gsa->pdop = atof(ptr);

  /* hdop */
  ptr = strchr(ptr, ',');
  if (ptr == NULL) return 0;
  ptr++;
  gsa->hdop = atof(ptr);

  /* vdop */
  ptr = strchr(ptr, ',');
  if (ptr == NULL) return 0;
  ptr++;
  gsa->vdop = atof(ptr);

  gsa->valid = 1;

  return 1;
}

int GPS_Parse_GST(GPS_gst *gst, char *line, TIMEVAL stamp)
{
  char *ptr;

  if(gst == NULL) return 0;
  gst->valid = 0;

  /* WARNING! By now we consider available time = measured time */
  /*Time_InitVoid(&gsa->meas_time);*/
  gst->meas_time  = stamp;
  gst->avail_time = stamp;
  gst->meas_id += 1;

  if(!GPS_CompChecksum(line)) return 0;

  /* UTC */
  ptr = strchr(line, ',');
  if (ptr == NULL) return 0;
  ptr++;
  gst->utc = ConvUTC_To_Timeval(ptr);

  /* RMS */
  ptr = strchr(ptr, ',');
  if (ptr==NULL) return 0;
  ptr++;
  gst->rms = atof(ptr);

  /* Sigma major */
  ptr = strchr(ptr, ',');
  if (ptr==NULL) return 0;
  ptr++;
  gst->sigma_major = atof(ptr);

  /* Sigma minor */
  ptr = strchr(ptr, ',');
  if (ptr==NULL) return 0;
  ptr++;
  gst->sigma_minor = atof(ptr);

  /* Orientation */
  ptr = strchr(ptr, ',');
  if (ptr==NULL) return 0;
  ptr++;
  gst->orientation = atof(ptr);

  /* Sigma latitude */
  ptr = strchr(ptr, ',');
  if (ptr==NULL) return 0;
  ptr++;
  gst->sigma_lat = atof(ptr);

  /* Sigma longitude */
  ptr = strchr(ptr, ',');
  if (ptr==NULL) return 0;
  ptr++;
  gst->sigma_long = atof(ptr);

  /* Sigma altitude */
  ptr = strchr(ptr, ',');
  if (ptr==NULL) return 0;
  ptr++;
  gst->sigma_alt = atof(ptr);

  gst->valid = 1;

  return 1;
}


int GPS_Parse_RMC(GPS_rmc *rmc, char *line, TIMEVAL stamp)
{
  char *ptr;
  char *ptr_longlat;

  ptr = line;

  if(rmc == NULL) return 0;
  rmc->valid = 0;

  /* WARNING! By now we consider available time = measured time */
  /* Time_InitVoid(&rmc->meas_time);*/
  rmc->meas_time = stamp;
  rmc->avail_time = stamp;
  rmc->meas_id += 1;

  if(strlen(line) < 60) return 0;

  if(!GPS_CompChecksum(line)) return 0;

  /* UTC */
  ptr = strchr(line, ',');
  if (ptr==NULL) return 0;
  ptr++;
  rmc->utc = ConvUTC_To_Timeval(ptr);

  /* Status */
  ptr = strchr(ptr, ',');
  if (ptr==NULL) return 0;
  ptr++;
  rmc->status = (ptr[0] == 'A')?1:0;

  /* Latitude */
  ptr = strchr(ptr, ',');
  if (ptr==NULL) return 0;
  ptr++;
  ptr_longlat = ptr;

  /* Latitude orient */
  ptr = strchr(ptr, ',');
  if (ptr==NULL) return 0;
  ptr++;
  if(!ORIENT_VALID(ptr[0])) return 0;
  rmc->lat_orient = ptr[0];
  rmc->latitude = ConvDegMin_To_Radians(ptr_longlat, rmc->lat_orient);

  /* Longitude */
  ptr = strchr(ptr, ',');
  if (ptr==NULL) return 0;
  ptr++;
  ptr_longlat = ptr;

  /* Longitude orient */
  ptr = strchr(ptr, ',');
  if (ptr==NULL) return 0;
  ptr++;
  if(!ORIENT_VALID(ptr[0])) return 0;
  rmc->long_orient = ptr[0];
  rmc->longitude = ConvDegMin_To_Radians(ptr_longlat, rmc->long_orient);

  /* Speed */
  ptr = strchr(ptr, ',');
  if (ptr==NULL) return 0;
  ptr++;
  rmc->speed = KNOTS_TO_METERPERSEC( atof(ptr) );

  /* COG */
  ptr = strchr(ptr, ',');
  if (ptr==NULL) return 0;
  ptr++;
  rmc->true_course = DEG2RAD( atof(ptr) );

  /* Date */
  ptr = strchr(ptr, ',');
  if (ptr==NULL) return 0;
  ptr++;
  rmc->date = atoi(ptr);

  /* Skip mag stuff */
  ptr = strchr(ptr, ',');
  if (ptr==NULL) return 0;
  ptr++;
  rmc->magnetic_variation = atof(ptr);

  /* Mode */
  ptr = strchr(ptr, ',');
  if (ptr==NULL) return 0;
  ptr++;
  rmc->magnetic_dir = ptr[0];

  /* Check the mode indicator */

  rmc->valid = 1;
  return 1;
}

/* [km] the differential gps signal parsing,
   here the validity of the message is set */
int GPS_Parse_GGA(GPS_gga *gga, char *line, TIMEVAL stamp)
{
  char *ptr;
  char *ptr_longlat;

  if(gga == NULL) return 0;
  gga->valid = 0;

  /* WARNING! By now we consider available time = measured time */
  /*  Time_InitVoid(&gga->meas_time);*/
  gga->meas_time  = stamp;
  gga->avail_time = stamp;
  gga->meas_id += 1;

  if(strlen(line) < 42) {
    fprintf(stderr, "gga: strlen=%d\n is less than 42!\n", strlen(line));
    return 0;
  }

  if(!GPS_CompChecksum(line)){
    fprintf(stderr, "gga: checksum error!\n");
    return 0;
  }

  ptr = strchr(line, ',');
  if (ptr==NULL) return 0;

  ptr = strchr(ptr, ',');
  if (ptr==NULL) return 0;
  ptr++;
  gga->utc = ConvUTC_To_Timeval(ptr);

  ptr = strchr(ptr, ',');
  if (ptr==NULL) return 0;
  ptr++;
  ptr_longlat = ptr;

  ptr = strchr(ptr, ',');
  if (ptr==NULL) return 0;
  ptr++;
  if(!ORIENT_VALID(ptr[0])){
    fprintf(stderr, "gga: orientation not valid!\n");
    return 0;
  }
  gga->lat_orient = ptr[0];
  gga->latitude = ConvDegMin_To_Radians(ptr_longlat, gga->lat_orient);

  ptr = strchr(ptr, ',');
  if (ptr==NULL) return 0;
  ptr++;
  ptr_longlat = ptr;

  ptr = strchr(ptr, ',');
  if (ptr==NULL) return 0;
  ptr++;
  if(!ORIENT_VALID(ptr[0])) return 0;
  gga->long_orient = ptr[0];
  gga->longitude = ConvDegMin_To_Radians(ptr_longlat, gga->long_orient);

  ptr = strchr(ptr, ',');
  if (ptr==NULL) return 0;
  ptr++;
  gga->gps_quality = atoi(ptr);

  ptr = strchr(ptr, ',');
  if (ptr==NULL) return 0;
  ptr++;
  gga->num_satellites = atoi(ptr);

  ptr = strchr(ptr, ',');
  if (ptr==NULL) return 0;
  ptr++;
  gga->hdop = atof(ptr);

  ptr = strchr(ptr, ',');
  if (ptr==NULL) return 0;
  ptr++;
  gga->altitude = atof(ptr);

  ptr = strchr(ptr, ',');
  if (ptr==NULL) return 0;
  ptr++;
  gga->altitude_unit = ptr[0];

  ptr = strchr(ptr, ',');
  if (ptr==NULL) return 0;
  ptr++;
  gga->geo_sea_level = atof(ptr);

  ptr = strchr(ptr, ',');
  if (ptr==NULL) return 0;
  ptr++;
  gga->geo_sep_unit = ptr[0];

  ptr = strchr(ptr, ',');
  if (ptr==NULL) return 0;
  ptr++;
  gga->data_age = atoi(ptr);

  ptr = strchr(ptr, ',');
  if (ptr==NULL) return 0;
  ptr++;
  gga->diff_station = atoi(ptr);

  gga->valid = 1;
  return 1;
}

int GPS_gps_parse_gga(char * line, TIMEVAL stamp)
{
  if(m_gpgga == NULL) GPS_gps_alloc();
  return GPS_Parse_GGA(m_gpgga,line,stamp);
}

int GPS_gps_parse_rmc(char * line, TIMEVAL stamp)
{
  if(m_gprmc == NULL) GPS_gps_alloc();
  return GPS_Parse_RMC(m_gprmc,line,stamp);
}

inline TIMEVAL GPS_GetAvailStampGGA()
{
  return m_gpgga->avail_time;
}

inline TIMEVAL GPS_GetAvailStampRMC()
{
  return m_gprmc->avail_time;
}

inline TIMEVAL GPS_GetMeasStampGGA()
{
  return m_gpgga->meas_time;
}

inline TIMEVAL GPS_GetMeasStampRMC()
{
  return m_gprmc->meas_time;
}

inline TIMEVAL GPS_GetUTCGGA()
{
  return m_gpgga->utc;
}

inline TIMEVAL GPS_GetUTCRMC()
{
  return m_gprmc->utc;
}

inline void GPS_SetMeasTimeStampGGA(TIMEVAL stamp)
{
  m_gpgga->meas_time = stamp;
}

inline void GPS_SetMeasTimeStampRMC(TIMEVAL stamp)
{
  m_gprmc->meas_time = stamp;
}

inline int GPS_GetIDRMC()
{
  return m_gprmc->meas_id;
}

void GPS_gps_alloc()
{
  static int first = 1;

  if(first)
  {
    m_gpgga = (GPS_gga *) malloc(sizeof(GPS_gga));
    m_gprmc = (GPS_rmc *) malloc(sizeof(GPS_rmc));

    if(m_gpgga == NULL || m_gprmc == NULL){
      fprintf(stderr, "GPS_gps_alloc -> Could not alloc memory!\n");
      exit(EXIT_FAILURE);
    }

    bzero(m_gpgga, sizeof(m_gpgga));
    bzero(m_gprmc, sizeof(m_gprmc));

    first = 0;
  }
}

void GPS_PrintGSA(GPS_gsa *gsa)
{
  int i;

  EPRINT("*** GPS_gsa struct\n");
  EPRINT("Number of the GPS unit: %i\n", gsa->nr);
  EPRINT("Measurement id: %i\n", gsa->meas_id);
  EPRINT("GPS quality: %s (%i)\n",GPS_GPS_FIX[gsa->gps_quality], gsa->gps_quality);
  EPRINT("PNR satellites:\n");
  for(i = 0; i < 12; i++)  EPRINT("%i, ", gsa->PRN[i]);

  EPRINT("\nPosition dilution of precision: %.2f\n",gsa->pdop);
  EPRINT("Horizontal dilution of precision: %.2f\n",gsa->hdop);
  EPRINT("Vertical dilution of precision: %.2f\n",gsa->vdop);
  EPRINT("Time of the measure: %.6f\n",Time_FromTimeval(gsa->meas_time));
  EPRINT("Time the measure is available: %.6f\n",Time_FromTimeval(gsa->avail_time));
}

void PrintGGA(GPS_gga *gga)
{
  EPRINT("*** GPS_gga struct\n");
  EPRINT("Number of the GPS unit: %i\n", gga->nr);
  EPRINT("Measurement id: %i\n", gga->meas_id);
  EPRINT("UTC: %.6f\n", Time_FromTimeval(gga->utc));
  EPRINT("Latitude: %.7f\n", gga->latitude);
  EPRINT("Lat orient: %c\n",gga->lat_orient);
  EPRINT("Longitude: %.7f\n",gga->longitude);
  EPRINT("Long orient: %c\n",gga->long_orient);
  EPRINT("GPS quality: %s (%i)\n",GPS_GPS_FIX[gga->gps_quality], gga->gps_quality);
  EPRINT("Num satellites: %i\n",gga->num_satellites);
  EPRINT("Horizontal dilution of precision: %.2f\n",gga->hdop);
  EPRINT("Altitude: %.1f\n",gga->altitude);
  EPRINT("Altitude unit: %c\n",gga->altitude_unit);
  EPRINT("Geoidal separation: %.1f\n",gga->geo_sea_level);
  EPRINT("Geoidal separation units: %c\n",gga->geo_sep_unit);
  EPRINT("Age of differential GPS data (time in sec since last update): %i\n",gga->data_age);
  EPRINT("Differential Ref. Station ID: %i\n",gga->diff_station);
  EPRINT("Time of the measure: %.6f\n",Time_FromTimeval(gga->meas_time));
  EPRINT("Time the measure is available: %.6f\n",Time_FromTimeval(gga->avail_time));
}

void GPS_PrintGGA(GPS_gga *gga)
{
  assert(m_gpgga != NULL);
  PrintGGA(m_gpgga);
}

void PrintRMC(GPS_rmc *rmc)
{
  EPRINT("*** GPS_rmc struct\n");
  EPRINT("Number of the GPS unit: %i\n", rmc->nr);
  EPRINT("Measurement id: %i\n", rmc->meas_id);
  EPRINT("UTC: %.6f\n", Time_FromTimeval(rmc->utc));
  EPRINT("Valid: %i\n", rmc->valid);
  EPRINT("Status: %i\n", rmc->status);
  EPRINT("Latitude: %.7f (%.7f deg)\n", rmc->latitude, RAD2DEG(rmc->latitude));
  EPRINT("Lat orient: %c\n",rmc->lat_orient);
  EPRINT("Longitude: %.7f (%.7f deg)\n", rmc->longitude, RAD2DEG(rmc->longitude));
  EPRINT("Long orient: %c\n",rmc->long_orient);
  EPRINT("Speed [m/s]: %f\n",rmc->speed);
  EPRINT("Heading to north: %f [rad] (%.6f deg)\n",rmc->true_course, RAD2DEG(rmc->true_course));
  EPRINT("UT Date: %i\n",rmc->date);
  EPRINT("Magnetic variation: %f\n",rmc->magnetic_variation);
  EPRINT("Magnetic direction: %c\n",rmc->magnetic_dir);
  EPRINT("Mode: %c\n",rmc->mode);
  EPRINT("Time of the measure: %.6f\n",Time_FromTimeval(rmc->meas_time));
  EPRINT("Time the measure is available: %.6f\n",Time_FromTimeval(rmc->avail_time));
}

void GPS_PrintRMC()
{
  assert(m_gprmc != NULL);
  PrintRMC(m_gprmc);
}

void GPS_InitVoidGSA(GPS_gsa *strct)
{
  int i;

  strct->valid = 0;
  strct->nr = -1;
  strct->meas_id = -1;
  strct->mode = 'V';
  strct->gps_quality = 0;
  for(i = 0; i < 12; i++) strct->PRN[i] = -1;
  strct->pdop = -1.0;
  strct->hdop = -1.0;
  strct->vdop = -1.0;

  Time_InitVoid(&strct->meas_time);
  Time_InitVoid(&strct->avail_time);
}

void GPS_InitVoidGST(GPS_gst *strct)
{
  strct->valid = 0;
  strct->nr = -1;
  strct->meas_id = -1;

  Time_InitVoid(&strct->utc);

  strct->rms = -1;
  strct->sigma_major = -1;
  strct->sigma_minor = -1;
  strct->orientation = -1;
  strct->sigma_lat = -1;
  strct->sigma_long = -1;
  strct->sigma_alt = -1;

  Time_InitVoid(&strct->meas_time);
  Time_InitVoid(&strct->avail_time);
}

void GPS_InitVoidGGA(GPS_gga *strct)
{
  strct->valid   = 0;
  strct->nr      = -1;
  strct->meas_id = -1;
  Time_InitVoid(&strct->utc);
  strct->latitude    = -1;
  strct->longitude   = -1;
  strct->lat_orient  = 'Z';
  strct->long_orient = 'Z';
  strct->gps_quality = -1;
  strct->num_satellites = -1;
  strct->hdop           = -1;
  strct->altitude       = -1;
  strct->altitude_unit  = 'Z';
  strct->geo_sea_level  = -1;
  strct->geo_sep_unit   = 'Z';
  strct->data_age       = -1;
  strct->diff_station   = -1;
  Time_InitVoid(&strct->meas_time);
  Time_InitVoid(&strct->avail_time);
}

void GPS_InitVoidRMC(GPS_rmc *strct)
{
  strct->valid    = 0;
  strct->status   = -1;
  strct->nr       = -1;
  strct->meas_id  = -1;
  strct->status   = -1;
  Time_InitVoid(&strct->utc);
  strct->latitude    = -1;
  strct->longitude   = -1;
  strct->lat_orient  = 'Z';
  strct->long_orient = 'Z';
  strct->speed       = -1;
  strct->true_course = -1;
  strct->date        = -1;
  strct->mode        = 'Z';
  strct->magnetic_variation = -1.0;;
  strct->magnetic_dir = 'Z';

  Time_InitVoid(&strct->meas_time);
  Time_InitVoid(&strct->avail_time);
}

void GPS_DumpGGA(FILE *strm, GPS_gga *gga)
{
  fprintf(strm,"%i%c%f%c%f%c%c%c%i%c%i%c%f%c%.15f%c%.15f%c",
          MODULE_NAME_NMEA_GGA,
          LOG_SEP,
          Time_FromTimeval(gga->meas_time),
          LOG_SEP,
          Time_FromTimeval(gga->avail_time),
          LOG_SEP,
          'G',
          LOG_SEP,
          gga->valid,
          LOG_SEP,
          gga->meas_id,
          LOG_SEP,
          Time_FromTimeval(gga->utc),
          LOG_SEP,
          gga->latitude,
          LOG_SEP,
          gga->longitude,
          LOG_SEP);

  fprintf(strm,"%i%c%f%c%i%c%f%c%f%c%i%c%i",
          gga->num_satellites,
          LOG_SEP,
          gga->hdop,
          LOG_SEP,
          gga->gps_quality,
          LOG_SEP,
          gga->altitude,
          LOG_SEP,
          gga->geo_sea_level,
          LOG_SEP,
          gga->data_age,
          LOG_SEP,
          gga->diff_station);
}

void GPS_DumpRMC(FILE *strm, GPS_rmc *rmc)
{
  fprintf(strm,"%i%c%f%c%f%c%c%c%i%c%i%c%i%c%f%c%.15f%c%.15f%c",
          MODULE_NAME_NMEA_RMC,
          LOG_SEP,
          Time_FromTimeval(rmc->meas_time),
          LOG_SEP,
          Time_FromTimeval(rmc->avail_time),
          LOG_SEP,
          'R',
          LOG_SEP,
          rmc->valid,
          LOG_SEP,
          rmc->status,
          LOG_SEP,
          rmc->meas_id,
          LOG_SEP,
          Time_FromTimeval(rmc->utc),
          LOG_SEP,
          rmc->latitude,
          LOG_SEP,
          rmc->longitude,
          LOG_SEP);

  fprintf(strm,"%f%c%f%c%i%c%f%c%c",
          rmc->speed,
          LOG_SEP,
          rmc->true_course,
          LOG_SEP,
          rmc->date,
          LOG_SEP,
          rmc->magnetic_variation,
          LOG_SEP,
          rmc->magnetic_dir);
}

void GPS_DumpGSA(FILE *strm, GPS_gsa *gsa)
{
  int i;

  fprintf(strm,"%i%c%f%c%f%c%c%c%i%c%i%c%c%c%i",
          MODULE_NAME_NMEA_GSA,
          LOG_SEP,
          Time_FromTimeval(gsa->meas_time),
          LOG_SEP,
          Time_FromTimeval(gsa->avail_time),
          LOG_SEP,
          'S',
          LOG_SEP,
          gsa->valid,
          LOG_SEP,
          gsa->meas_id,
          LOG_SEP,
          gsa->mode,
          LOG_SEP,
          gsa->gps_quality);


  for(i = 0; i < 12; i++)
    fprintf(strm,"%c%i",LOG_SEP,gsa->PRN[i]);

  fprintf(strm,"%c%f%c%f%c%f",
          LOG_SEP,
          gsa->pdop,
          LOG_SEP,
          gsa->hdop,
          LOG_SEP,
          gsa->vdop);
}

void GPS_DumpGST(FILE *strm, GPS_gst *gst)
{
  fprintf(strm,"%i%c%f%c%f%c%c%c%i%c%i%c",
          MODULE_NAME_NMEA_GST,
          LOG_SEP,
          Time_FromTimeval(gst->meas_time),
          LOG_SEP,
          Time_FromTimeval(gst->avail_time),
          LOG_SEP,
          'T',
          LOG_SEP,
          gst->valid,
          LOG_SEP,
          gst->meas_id,
          LOG_SEP);

  fprintf(strm,"%f%c%f%c%f%c%f%c%f%c%f%c%f%c%f",
          Time_FromTimeval(gst->utc),
          LOG_SEP,
          gst->rms,
          LOG_SEP,
          gst->sigma_major,
          LOG_SEP,
          gst->sigma_minor,
          LOG_SEP,
          gst->orientation,
          LOG_SEP,
          gst->sigma_lat,
          LOG_SEP,
          gst->sigma_long,
          LOG_SEP,
          gst->sigma_alt);
}

/* return 0 if error */
int GPS_ReadGSAFromFile(FILE *input, GPS_gsa *strct)
{
  double tv_meas, tv_avail;
  char   buf[512], gsa;
  int    msg_type;
  int    items,i;

  if(feof(input)) return 0;

  if(DUMP_FMT_GSA[0] == '#')
  {
    strcpy(DUMP_FMT_GSA,GPS_INT_FMT);
    strcat(DUMP_FMT_GSA,GPS_REAL_FMT);
    strcat(DUMP_FMT_GSA,GPS_REAL_FMT);
    strcat(DUMP_FMT_GSA,GPS_CHAR_FMT);
    strcat(DUMP_FMT_GSA,GPS_INT_FMT);
    strcat(DUMP_FMT_GSA,GPS_INT_FMT);
    strcat(DUMP_FMT_GSA,GPS_CHAR_FMT);
    strcat(DUMP_FMT_GSA,GPS_INT_FMT);
    for(i = 0; i < 12; i++)
      strcat(DUMP_FMT_GSA,GPS_INT_FMT);
    strcat(DUMP_FMT_GSA,GPS_REAL_FMT);
    strcat(DUMP_FMT_GSA,GPS_REAL_FMT);
    strcat(DUMP_FMT_GSA,GPS_REAL_FMT);
    strcat(DUMP_FMT_GSA,"\n");
  }

  fgets(buf,512,input);
  items = sscanf(buf,DUMP_FMT_GSA,
                 &msg_type,
                 &tv_meas,
                 &tv_avail,
                 &gsa,
                 &strct->valid,
                 &strct->meas_id,
                 &strct->mode,
                 &strct->gps_quality,
                 &strct->PRN[0],&strct->PRN[1], &strct->PRN[2],
                 &strct->PRN[3], &strct->PRN[4], &strct->PRN[5],
                 &strct->PRN[6], &strct->PRN[7], &strct->PRN[8],
                 &strct->PRN[9], &strct->PRN[10], &strct->PRN[11],
                 &strct->pdop,
                 &strct->hdop,
                 &strct->vdop);

  if(items != 23){
    strct->valid = 0;
    EERR("GPS_ReadGSAFromFile -> Could not read all items (%i)\n", items);
    return 0;
  }

  strct->meas_time  = Time_FromDbl(tv_meas);
  strct->avail_time = Time_FromDbl(tv_avail);

  return -1;
}

/* return 0 if error */
int GPS_ReadGSTFromFile(FILE *input, GPS_gst *strct)
{
  double tv_meas, tv_avail, tv_utc;
  char   buf[512], gst;
  int    msg_type;
  int    items;

  if(feof(input)) return 0;

  if(DUMP_FMT_GST[0] == '#')
  {
    strcpy(DUMP_FMT_GST,GPS_INT_FMT);
    strcat(DUMP_FMT_GST,GPS_REAL_FMT);
    strcat(DUMP_FMT_GST,GPS_REAL_FMT);
    strcat(DUMP_FMT_GST,GPS_CHAR_FMT);
    strcat(DUMP_FMT_GST,GPS_INT_FMT);
    strcat(DUMP_FMT_GST,GPS_INT_FMT);

    strcat(DUMP_FMT_GST,GPS_REAL_FMT);
    strcat(DUMP_FMT_GST,GPS_REAL_FMT);
    strcat(DUMP_FMT_GST,GPS_REAL_FMT);
    strcat(DUMP_FMT_GST,GPS_REAL_FMT);
    strcat(DUMP_FMT_GST,GPS_REAL_FMT);
    strcat(DUMP_FMT_GST,GPS_REAL_FMT);
    strcat(DUMP_FMT_GST,GPS_REAL_FMT);
    strcat(DUMP_FMT_GST,GPS_REAL_FMT);
    strcat(DUMP_FMT_GST,"\n");
  }

  fgets(buf,512,input);
  items = sscanf(buf,DUMP_FMT_GST,
                 &msg_type,
                 &tv_meas,
                 &tv_avail,
                 &gst,
                 &strct->valid,
                 &strct->meas_id,
                 &tv_utc,
                 &strct->rms,
                 &strct->sigma_major,
                 &strct->sigma_minor,
                 &strct->orientation,
                 &strct->sigma_lat,
                 &strct->sigma_long,
                 &strct->sigma_alt);

  if(items != 14){
    strct->valid = 0;
    EERR("GPS_ReadGSTFromFile -> Could not read all items (%i)\n", items);
    return 0;
  }

  strct->meas_time  = Time_FromDbl(tv_meas);
  strct->avail_time = Time_FromDbl(tv_avail);
  strct->utc        = Time_FromDbl(tv_utc);

  return -1;
}


/* return 0 if error */
int GPS_ReadGGAFromFile(FILE *input, GPS_gga *strct)
{
  double tv_meas, tv_avail, tv_utc;
  char   buf[512], gga;
  int    msg_type;
  int    items;

  if(feof(input)) return 0;

  if(DUMP_FMT_GGA[0] == '#')
  {
    strcpy(DUMP_FMT_GGA,GPS_INT_FMT);
    strcat(DUMP_FMT_GGA,GPS_REAL_FMT);
    strcat(DUMP_FMT_GGA,GPS_REAL_FMT);
    strcat(DUMP_FMT_GGA,GPS_CHAR_FMT);
    strcat(DUMP_FMT_GGA,GPS_INT_FMT);
    strcat(DUMP_FMT_GGA,GPS_INT_FMT);
    strcat(DUMP_FMT_GGA,GPS_REAL_FMT);
    strcat(DUMP_FMT_GGA,GPS_REAL_FMT);
    strcat(DUMP_FMT_GGA,GPS_REAL_FMT);
    strcat(DUMP_FMT_GGA,GPS_INT_FMT);
    strcat(DUMP_FMT_GGA,GPS_REAL_FMT);
    strcat(DUMP_FMT_GGA,GPS_INT_FMT);
    strcat(DUMP_FMT_GGA,GPS_REAL_FMT);
    strcat(DUMP_FMT_GGA,GPS_REAL_FMT);
    strcat(DUMP_FMT_GGA,GPS_INT_FMT);
    strcat(DUMP_FMT_GGA,GPS_INT_FMT);
    strcat(DUMP_FMT_GGA,"\n");
  }

  fgets(buf,512,input);
  items = sscanf(buf,DUMP_FMT_GGA,
                 &msg_type,
                 &tv_meas,
                 &tv_avail,
                 &gga,
                 &strct->valid,
                 &strct->meas_id,
                 &tv_utc,
                 &strct->latitude,
                 &strct->longitude,
                 &strct->num_satellites,
                 &strct->hdop,
                 &strct->gps_quality,
                 &strct->altitude,
                 &strct->geo_sea_level,
                 &strct->data_age,
                 &strct->diff_station);

  if(items != 16){
    strct->valid = 0;
    return 0;
  }

  strct->meas_time = Time_FromDbl(tv_meas);
  strct->avail_time = Time_FromDbl(tv_avail);
  strct->utc = Time_FromDbl(tv_utc);

  return -1;
}

/* return 0 if error */
int GPS_ReadRMCFromFile(FILE *input, GPS_rmc *strct)
{
  double tv_meas, tv_avail, tv_utc;
  char   buf[512], rmc;
  int    msg_type;
  int    items;

  if(feof(input)) return 0;

  if(DUMP_FMT_RMC[0] == '#')
  {
    strcpy(DUMP_FMT_RMC,GPS_INT_FMT);
    strcat(DUMP_FMT_RMC,GPS_REAL_FMT);
    strcat(DUMP_FMT_RMC,GPS_REAL_FMT);
    strcat(DUMP_FMT_RMC,GPS_CHAR_FMT);
    strcat(DUMP_FMT_RMC,GPS_INT_FMT);
    strcat(DUMP_FMT_RMC,GPS_INT_FMT);
    strcat(DUMP_FMT_RMC,GPS_INT_FMT);
    strcat(DUMP_FMT_RMC,GPS_REAL_FMT);
    strcat(DUMP_FMT_RMC,GPS_REAL_FMT);
    strcat(DUMP_FMT_RMC,GPS_REAL_FMT);
    strcat(DUMP_FMT_RMC,GPS_REAL_FMT);
    strcat(DUMP_FMT_RMC,GPS_REAL_FMT);
    strcat(DUMP_FMT_RMC,GPS_INT_FMT);
    strcat(DUMP_FMT_RMC,GPS_REAL_FMT);
    strcat(DUMP_FMT_RMC,GPS_CHAR_FMT);
    strcat(DUMP_FMT_RMC,"\n");
  }

  fgets(buf,512,input);
  items = sscanf(buf,DUMP_FMT_RMC,
                 &msg_type,
                 &tv_meas,
                 &tv_avail,
                 &rmc,
                 &strct->valid,
                 &strct->status,
                 &strct->meas_id,
                 &tv_utc,
                 &strct->latitude,
                 &strct->longitude,
                 &strct->speed,
                 &strct->true_course,
                 &strct->date,
                 &strct->magnetic_variation,
                 &strct->magnetic_dir);

  if(items != 15){
    strct->valid = 0;
    return 0;
  }

  strct->meas_time  = Time_FromDbl(tv_meas);
  strct->avail_time = Time_FromDbl(tv_avail);
  strct->utc = Time_FromDbl(tv_utc);

  return -1;
}



