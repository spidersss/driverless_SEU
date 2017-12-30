#ifndef GPS_COM_H_
#define GPS_COM_H_

#include"serial_open.h"

struct GPS_MESSAGES{
	double lat;
	double lon;
	float yaw;
	float vel_n;
	float vel_e;
};

int gps_find_head(char buf_gps[], int cur, int len_gps);
void gps_get_mes(char buf_gps[], int cur, struct GPS_MESSAGES* gps_mes);
void gps_com(char buf_gps[], int len_gps, struct GPS_MESSAGES* gps_mes);
void gps_init(char buf_gps[], int len_gps, struct GPS_MESSAGES* gps_mes);
int gps_check(struct GPS_MESSAGES* gps_mes);

#endif
