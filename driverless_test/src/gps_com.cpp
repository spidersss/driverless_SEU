#include"gps_com.h"


int gps_find_head(char* buf_gps, int cur, int len_gps){
	if(buf_gps[cur] == 0x14 && buf_gps[cur+1] == 0x64 && cur + 103 < len_gps)
		return 1;
	else 
		return 0;
}

void gps_get_mes(char* buf_gps, int cur, struct GPS_MESSAGES* gps){
	char lat_[8], lon_[8], yaw_[4], vel_n_[4], vel_e_[4];
	int j = cur + 15 + 1;//altitude 
	int k = 0;
	for(; k < 8; k+=1){
		lat_[k] = buf_gps[j];
		j += 1;
	}
	k = 0;
	for(; k < 8; k+=1){
		lon_[k] = buf_gps[j];
		j += 1;
	}
	k = 0;
	j += 8;
	for(; k < 4; k+=1){
		vel_n_[k] = buf_gps[j];
		j += 1;
	}
	k = 0;
	for(; k < 4; k+=1){
		vel_e_[k] = buf_gps[j];
		j += 1;
	}
	k = 0;
	j += 4*7;
	for(; k < 4; k+=1){
		yaw_[k] = buf_gps[j];
		j += 1;
	}
	gps->lat = *((double*)lat_) * 180 / pi;
	gps->lon = *((double*)lon_) * 180 / pi;
	gps->vel_n = *((float*)vel_n_) * 3.6;
	gps->vel_e = *((float*)vel_e_) * 3.6;
	gps->yaw = *((float*)yaw_) * 180 / pi;
}

void gps_com(char* buf_gps, int len_gps, struct GPS_MESSAGES* gps){
	int i = 0;
	for(i = 0; i < len_gps; i++){
		if(gps_find_head(buf_gps, i, len_gps)){//find the head 
			gps_get_mes(buf_gps, i, gps);
		}
	}
}

void gps_init(char* buf_gps, int len_gps, struct GPS_MESSAGES* gps){
	int i = 0;
	for(i = 0; i < len_gps; i++){
		if(gps_find_head(buf_gps, i, len_gps)){//find the head 
			gps_get_mes(buf_gps, i, gps);
		}
	}
}

int gps_check(struct GPS_MESSAGES* gps){
	if((abs(gps->lat) < 100 && abs(gps->lon) < 200) && ((abs(gps->vel_n)<100)&&(abs(gps->vel_e)<100)&&(abs(gps->yaw)<400)) && gps->yaw < 360)
		return 1;
	else
		return 0;
}
