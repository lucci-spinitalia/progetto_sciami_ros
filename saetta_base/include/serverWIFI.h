/* 
 * File:   serverWIFI.h
 * Author: erupter
 * Based on work by Daniela Carboni
 * Created on December 4, 2012, 11:25 AM
 */

#ifndef SERVERWIFI_H
#define	SERVERWIFI_H

namespace SaettaServerWifi
{
    #define WIFI_PORT 5069
    #define MSG_STOP 1

    struct payload_pkg_trajectory {
            float  v; //velocità lineare
            float  w; //velocità angolare
    };

    typedef unsigned char uint8;

    typedef enum{
            SPEED,
            MSG,
            KIN_SPEEDS,
            KIN_PARAMS
    }pkg_t;

    typedef struct {
            float v;
            float w;
    } wifi_vel;

    typedef struct {
            int msg;
    } wifi_msg;


    class WifiTx {
        public:
            //WifiTx();
            //WifiTx(const WifiTx& orig);
            //virtual ~WifiTx();
            int init_wifi(char *ip);
            int send_wifi_vel(float vlin, float omega);
            int send_wifi_vel(int sock, float vlin, float omega);
            int send_wifi_msg(int m);
            int send_wifi_msg(int sock, int m);

        private:
            int _socket;
            int _init;

    };
}






#endif	/* SERVERWIFI_H */

