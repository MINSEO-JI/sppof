#include <iostream>

#include <string.h>
#include <stdexcept>
#include <stdio.h>
#include <stdlib.h>

#include <cstring>
#include <cstdlib>

#include <cmath>
#include <iomanip>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <sys/types.h>
#include <valarray>
#include <netinet/in.h>
#include <sys/time.h>
#include "/home/gps/c_library_v2/common/mavlink.h"

#define MAVLINK_MAX_PACKET_LEN 1024 
#define MAV_COMP_ID_GPS 220
#define EARTH_RADIUS (6378137.0f)

typedef float real_T;

const int BUFFER_SIZE = 2048;



struct GeoPoint
{
    double latitude = 0, longitude = 0;
    float altitude = 0;

    GeoPoint()
    {
    }

    GeoPoint(double latitude_val, double longitude_val, float altitude_val)
    {
        set(latitude_val, longitude_val, altitude_val);
    }

    void set(double latitude_val, double longitude_val, float altitude_val)
    {
        latitude = latitude_val, longitude = longitude_val;
        altitude = altitude_val;
    }

    friend std::ostream& operator<<(std::ostream& os, GeoPoint const& g)
    {
        return os << "[" << g.latitude << ", " << g.longitude << ", " << g.altitude << "]";
    }

};


class Vector3r 
{
public:
    double latitude;
    double longitude;
    float  altitude;
    double x;
    double y;
    double z;

    Vector3r() : x(0.0), y(0.0), z(0.0) {}

    Vector3r(double _latitude, double _longitude, float _altitude) : latitude(_latitude), longitude(_longitude), altitude(_altitude) 
    {
        x = _latitude;
        y = _longitude;
        z = _altitude;
    }

    double norm() const 
    {
        return std::sqrt(x * x + y * y + z * z);
    }

    double& operator()(int index) 
    {
        if (index == 0) {
            return x;
        } else if (index == 1) {
            return y;
        } else if (index == 2) {
            return z;
        } else {
            throw std::out_of_range("Invalid index");
        }
    }

    const double& operator()(int index) const 
    {
        if (index == 0) {
            return x;
        } else if (index == 1) {
            return y;
        } else if (index == 2) {
            return z;
        } else {
            throw std::out_of_range("Invalid index");
        }
    }

    Vector3r operator-(const Vector3r& other) const 
    {
        return Vector3r(x - other.x, y - other.y, z - other.z);
    }

    Vector3r operator/(double scalar) const 
    {
        return Vector3r(x / scalar, y / scalar, z / scalar);
    }

    Vector3r operator*(double scalar) const 
    {
        return Vector3r(x * scalar, y * scalar, z * scalar);
    }

    friend Vector3r operator*(double scalar, const Vector3r& vec) 
    {
        return Vector3r(vec.x * scalar, vec.y * scalar, vec.z * scalar);
    }

    Vector3r& operator-=(const Vector3r& other) 
    {
        x -= other.x;
        y -= other.y;
        z -= other.z;
        return *this;
    }
};


static constexpr double degreesToRadians(double degrees) 
{
    // 도 단위를 라디안단위로 변환
    return degrees * M_PI / 180.0;

    // 90도가 라디안단위로 변환하여 1.5708로 출력하는 함수
    // int main() {
    // double degrees = 90.0;
    // double radians = degreesToRadians(degrees);
    // std::cout << "90 degrees is " << radians << " radians." << std::endl;
    // return 0;
    // }
}

static constexpr double radiansToDegrees(double radians) 
{
    // 라디안 단위츨 도 단위로 변환
    return radians * 180.0 / M_PI;
}

class EarthUtils 
{
public:
    static Vector3r GeodeticToNedFast(const GeoPoint& geo, const GeoPoint& home) 
    {
        // GPS좌표계를 NED좌표로 변환하는 함수

        double d_lat_rad = degreesToRadians(geo.latitude - home.latitude);
        double d_lon_rad = degreesToRadians(geo.longitude - home.longitude);

        real_T x = d_lat_rad * EARTH_RADIUS;
        real_T y = d_lon_rad * EARTH_RADIUS * cos(degreesToRadians(home.latitude));
        real_T d_alt = home.altitude - geo.altitude;

        // // 좌표 축소
        // const double scale_factor = 0.00001;
        // x *= scale_factor;
        // y *= scale_factor;

        return Vector3r(x, y, d_alt);

    }

    static GeoPoint nedToGeodeticFast(const Vector3r& local, const GeoPoint& home) 
    {
        // NED 좌표를 GPS좌표계로 변환하는 함수
        GeoPoint r;
        double d_lat = local.x / EARTH_RADIUS;
        double d_lon = local.y / (EARTH_RADIUS * cos(degreesToRadians(home.latitude)));
        r.latitude = home.latitude + radiansToDegrees(d_lat);
        r.longitude = home.longitude + radiansToDegrees(d_lon);
        r.altitude = home.altitude - local.z;
        return r;
    }

};

// GeoPoint Spoofing(const GeoPoint& receivedData, const GeoPoint& protect_point)
// {
//     // FILE* pFile_true = fopen("/home/gps/AirSim-1.8.0-linux/Unreal/Environments/Blocks/true2.txt", "a");
//     // FILE* pFile_spoof = fopen("/home/gps/AirSim-1.8.0-linux/Unreal/Environments/Blocks/spoof2.txt", "a");
//     GeoPoint spoof_position = receivedData;

//     Vector3r curr_ned = EarthUtils::GeodeticToNedFast(receivedData,protect_point);
//     std::cout << "Current NED: " << std::fixed << std::setprecision(4) << curr_ned.x << ", " << curr_ned.y << ", " << curr_ned.z << std::endl;

//     float off_distance = 50;
//     float off_distance_big = 70;
//     GeoPoint protect_point_lla;

//     float alpha = 5.0;
//     Vector3r sp_vector = curr_ned;
//     // sp_vector(2) = 0;

//     float n_sp =sp_vector.norm();

//     printf("Vector norm: %3f\n", n_sp);

//     Vector3r spoof_ned;            

//     // fprintf(pFile_true,"%f %f %f\n",curr_ned(0),curr_ned(1),curr_ned(2)); 
//     if(n_sp < off_distance_big)
//     {
//         Vector3r delta_pose_big = alpha * sp_vector/n_sp * (off_distance_big-n_sp); 
//         spoof_ned = curr_ned - delta_pose_big;
//         std::cout << "Spoofing Delta Pose Big: " << std::fixed << std::setprecision(4) << delta_pose_big.x << std::endl;
//         if (n_sp < off_distance)  
//         {
//             Vector3r delta_pose = alpha * sp_vector / n_sp * (off_distance - n_sp);
//             std::cout << "Delta Pose: " << std::fixed << std::setprecision(4) << delta_pose.x << std::endl;
//             // spoof_ned = curr_ned - delta_pose;
//         }
//         spoof_position = EarthUtils::nedToGeodeticFast(spoof_ned,protect_point);
//     }
//     // else
//         // spoof_ned = curr_ned;

//     //  fprintf(pFile_spoof, "%f %f %f\n", spoof_ned(0), spoof_ned(1), spoof_ned(2));

//     // fclose(pFile_true);
//     // fclose(pFile_spoof);

//     return spoof_position;
// }

GeoPoint Spoofing(const GeoPoint& receivedData, const GeoPoint& protect_point)
{
     return receivedData;
    FILE* pFile_true = fopen("/home/gps/drone_simulator/AirSim-1.8.1/Unreal/Environments/Blocks/true6.txt", "a");
    FILE* pFile_spoof = fopen("/home/gps/drone_simulator/AirSim-1.8.1/Unreal/Environments/Blocks/spoof6.txt", "a");
    GeoPoint spoof_position = receivedData;

    Vector3r curr_ned = EarthUtils::GeodeticToNedFast(receivedData, protect_point);
    std::cout << "Current NED: " << std::fixed << std::setprecision(4) << curr_ned.x << ", " << curr_ned.y << ", " << curr_ned.z << std::endl;

    float off_distance = 1000;
    float off_distance_big = 1000;
    GeoPoint protect_point_lla;

    float alpha = 5.0;
    Vector3r sp_vector = curr_ned;
    // sp_vector(2) = 0;

    float n_sp = sp_vector.norm();

    printf("Vector norm: %3f\n", n_sp);

    Vector3r spoof_ned;

    fprintf(pFile_true, "%f %f %f\n", curr_ned(0), curr_ned(1), curr_ned(2)); 

    if (n_sp < off_distance_big)
    {
        Vector3r delta_pose_big = alpha * sp_vector / n_sp * (off_distance_big - n_sp); 
        spoof_ned = curr_ned - delta_pose_big;
        std::cout << "Spoofing Delta Pose Big: " << std::fixed << std::setprecision(4) << delta_pose_big.x << std::endl;
        if (n_sp < off_distance)  
        {
            Vector3r delta_pose = alpha * sp_vector / n_sp * (off_distance - n_sp);
            std::cout << "Delta Pose: " << std::fixed << std::setprecision(4) << delta_pose.x << std::endl;
            spoof_ned = curr_ned - delta_pose;
        }
        spoof_position = EarthUtils::nedToGeodeticFast(spoof_ned, protect_point);
    }
    // else
    //     spoof_ned = curr_ned;

    fprintf(pFile_spoof, "%f %f %f\n", spoof_ned(0), spoof_ned(1), spoof_ned(2));

    fclose(pFile_true);
    fclose(pFile_spoof);

    return spoof_position;
}



struct spoofGPS
{
    uint64_t time_usec = 0;
    uint8_t fix_type = 0;
    int32_t lat = 0;
    int32_t lon = 0;
    int32_t alt = 0;
    uint16_t eph = 0;
    uint16_t epv = 0;
    uint16_t vel = 0;
    int16_t vn = 0;
    int16_t ve = 0;
    int16_t vd = 0;
    uint16_t cog = 0;
    uint8_t satellites_visible = 0;
    uint8_t id = 0;
};


int main() {
    int receiveSocket, sendSocket;
    struct sockaddr_in receiveAddr, cliaddr, sendrAddr;
    bool init_flag = false;
    GeoPoint protect_point;

    if ((receiveSocket = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("receive Socket creation failed");
        return -1; 
    }

    memset(&receiveAddr, 0, sizeof(receiveAddr));
    memset(&cliaddr, 0, sizeof(cliaddr));
    receiveAddr.sin_family = AF_INET;
    receiveAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    receiveAddr.sin_port = htons(19000);

    if (bind(receiveSocket, (const struct sockaddr *)&receiveAddr, sizeof(receiveAddr)) < 0) 
    {
        perror("Binding receive failed");
        return -1;
    }

    if ((sendSocket = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("send Socket creation failed");
        return -1;
    }

    memset(&sendrAddr, 0, sizeof(sendrAddr));
    sendrAddr.sin_family = AF_INET;
    sendrAddr.sin_port = htons(17000); 
    sendrAddr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);

    while (true) 
    {
        bool received_flag = false;
        bool send_flag = false;
        bool spoofer_flag = false;

        spoofGPS spof_gps; 
        uint8_t buf[MAVLINK_MAX_PACKET_LEN]; 
        socklen_t addr_len = sizeof(cliaddr);
        ssize_t recv_len = recvfrom(receiveSocket, buf, MAVLINK_MAX_PACKET_LEN, 0, (struct sockaddr *)&cliaddr, &addr_len);

        if (recv_len > 0) {
            std::cout << "Received size: " << recv_len << " bytes, Sent from: " << inet_ntoa(cliaddr.sin_addr) << ":" << ntohs(cliaddr.sin_port) << std::endl;
            std::cout << std::dec << std::endl;
            if (recv_len == sizeof(spoofGPS)) {
                memcpy(&spof_gps, buf, sizeof(spoofGPS));
                received_flag = true;
                printf("lat: %8f, lon: %8f \n",spof_gps.lat / 1E7,spof_gps.lon / 1E7 );
                // std::cout << "Decoded GPS data:" << std::endl;
                // std::cout << "  time_usec: " << spof_gps.time_usec << std::endl;
                // std::cout << "  lat: " << spof_gps.lat / 1E7 << std::endl;
                // std::cout << "  lon: " << spof_gps.lon / 1E7 << std::endl;
                // std::cout << "  alt: " << spof_gps.alt / 1000 << std::endl;
                // std::cout << "  vn: " << spof_gps.vn / 100.0 << " m/s" << std::endl;
                // std::cout << "  ve: " << spof_gps.ve / 100.0 << " m/s" << std::endl;
                // std::cout << "  vd: " << spof_gps.vd / 100.0 << " m/s" << std::endl;
                // std::cout << "  eph: " << spof_gps.eph / 100.0 << " m" << std::endl;
                // std::cout << "  epv: " << spof_gps.epv / 100.0 << " m" << std::endl;
                // std::cout << "  fix_type: " << static_cast<int>(spof_gps.fix_type) << std::endl;
                // std::cout << "  vel: " << spof_gps.vel / 100.0 << " m/s" << std::endl;
                // std::cout << "  cog: " << spof_gps.cog / 100.0 << " deg" << std::endl;
                // std::cout << "  satellites_visible: " << static_cast<int>(spof_gps.satellites_visible) << std::endl;
                if(!init_flag)
                {
                    protect_point.latitude = spof_gps.lat / 1E7;
                    protect_point.longitude = spof_gps.lon / 1E7;
                    protect_point.altitude= spof_gps.alt / 1000;
                    init_flag = true;
                }

            } else {
                std::cerr << "GPS data length is incorrect." << std::endl;
                received_flag = false;
            }
        }

        //spoofer
        if(received_flag && init_flag)
        {
            GeoPoint receivedData;
            receivedData.latitude = spof_gps.lat / 1E7;
            receivedData.longitude = spof_gps.lon / 1E7;
            receivedData.altitude= spof_gps.alt / 1000;

            // 방어하는 지역 좌표
            // 3번
            // protect_point.latitude = 47.6438255;
            // protect_point.longitude = -122.1394196;
            protect_point.latitude = 35.7213816;
            protect_point.longitude = 129.4808013;
            protect_point.altitude = 100.00000;

            GeoPoint position =  Spoofing( receivedData, protect_point);

            spoofer_flag = true;
            spof_gps.lat = position.latitude * 1e7;
            spof_gps.lon = position.longitude* 1e7;
            spof_gps.alt = position.altitude* 1e3;
            printf("update spoofing\n");
            
        }
        
        //send gps data
        if(received_flag && spoofer_flag)
        {
            // PX4로 보내는 것
            mavlink_message_t msg;
            uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

            mavlink_hil_gps_t hil_gps; 
            hil_gps.time_usec = spof_gps.time_usec;
            hil_gps.lat = spof_gps.lat;
            hil_gps.lon = spof_gps.lon;
            hil_gps.alt = spof_gps.alt; 
            hil_gps.vn = spof_gps.vn;
            hil_gps.ve = spof_gps.ve;
            hil_gps.vd = spof_gps.vd;
            hil_gps.eph = spof_gps.eph;
            hil_gps.epv = spof_gps.epv;
            hil_gps.fix_type = spof_gps.fix_type;
            hil_gps.vel = spof_gps.vel;
            hil_gps.cog = spof_gps.cog;
            hil_gps.satellites_visible = spof_gps.satellites_visible;

            mavlink_msg_hil_gps_encode(255, 200, &msg, &hil_gps);
            uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);

            if (sendto(sendSocket, buffer, len, 0, (const struct sockaddr*)&sendrAddr, sizeof(sendrAddr)) == -1) {
                std::cerr << "Failed to send data via UDP" << std::endl;
                ::close(sendSocket);
                send_flag = false;
                
            }else {
                std::cerr << "port : " << ntohs(sendrAddr.sin_port) << std::endl;
                send_flag = true;
            }
        }

    }

    close(receiveSocket);
    close(sendSocket);
    return 0;
}


