/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012-, Open Perception, Inc.
 *  Copyright (c) 2012,2015 The MITRE Corporation
 *  Copyright (c) 2017 Hesai Photonics Technology Co., Ltd
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */


#ifndef PCL_IO_Pandar_GRABBER_H_
#define PCL_IO_Pandar_GRABBER_H_

#include <pcl/io/grabber.h>
#include <pcl/io/impl/synchronized_queue.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <boost/asio.hpp>
#include <string>

#include "proto/lidar_frame.pb.h"

// Start of Packet & Packet Angle size , SOF 2bytes , Angle 2 bytes
#define HS_LIDAR_L40_SOP_ANGLE_SIZE (4)
// Unit size = distance(3bytes) + reflectivity(2bytes) for each Line
#define HS_LIDAR_L40_SERIAL_UNIT_SIZE (5)
// Unit num = Line 40 
#define HS_LIDAR_L40_UNIT_NUM (40)
// Every udp packet has 6 blocks
#define HS_LIDAR_L40_BLOCK_NUM (6)
// Block size = unit num * unit size + SOP + Angle
#define HS_LIDAR_L40_SERIAL_BLOCK_SIZE (HS_LIDAR_L40_SERIAL_UNIT_SIZE * HS_LIDAR_L40_UNIT_NUM + HS_LIDAR_L40_SOP_ANGLE_SIZE)
// Block tail = timestamp ( 4 bytes ) + factory num (2 bytes)
#define HS_LIDAR_L40_TIMESTAMP_SIZE (4)
#define HS_LIDAR_L40_FACTORY_SIZE (2)
#define HS_LIDAR_L40_RESERVED_SIZE (8)
#define HS_LIDAR_L40_ENGINE_VELOCITY (2)
#define HS_LIDAR_L40_TAIL_SIZE (HS_LIDAR_L40_TIMESTAMP_SIZE + HS_LIDAR_L40_FACTORY_SIZE + HS_LIDAR_L40_RESERVED_SIZE + HS_LIDAR_L40_ENGINE_VELOCITY)
// total packet size
#define HS_LIDAR_L40_PACKET_SIZE (HS_LIDAR_L40_SERIAL_BLOCK_SIZE * HS_LIDAR_L40_BLOCK_NUM + HS_LIDAR_L40_TAIL_SIZE)

    /*
     *   Type Definition
     */
    typedef struct HS_LIDAR_L40_Unit_s{
        int distance; // *2mm , real distance =  distance * 2 mm; max distance: (2^24 â€?1) * 2mm = 33554.43m
        unsigned short reflectivity; // reflectivity  
    }HS_LIDAR_L40_Unit;


    typedef struct HS_LIDAR_L40_Block_s{
        unsigned short sob;
        unsigned short Azimuth; // packet angle  ,Azimuth = RealAzimuth * 100
        HS_LIDAR_L40_Unit units[HS_LIDAR_L40_UNIT_NUM];
    }HS_LIDAR_L40_Block;

    typedef struct HS_LIDAR_L40_Packet_s{
        HS_LIDAR_L40_Block blocks[HS_LIDAR_L40_BLOCK_NUM];
        unsigned char reserved[HS_LIDAR_L40_RESERVED_SIZE];
        unsigned short engine_velocity; // real velocity = <engine_velocity> * 6/11 round/min
        unsigned int timestamp; // ms
        unsigned char factory[2];
    }HS_LIDAR_L40_Packet;


#define Pandar_Grabber_toRadians(x) ((x) * M_PI / 180.0)
/** \brief Grabber for the Hesai LiDAR (Pandar)
 * \author Philip Pi <piziwei@hesaitech.com>
 * \ingroup io
 */
namespace pcl
{
class PCL_EXPORTS  PandarGrabber : public Grabber
{
  public:
    /** \brief Signal used for a single sector
     *         Represents 1 corrected packet from the Pandar Hesai
     */
    typedef void
    (sig_cb_Hesai_Pandar_scan_raw_data) (const boost::shared_ptr<const hesai::Scan >&);

    /** \brief Signal used for a single sector
     *         Represents 1 corrected packet from the Pandar Hesai
     */
    typedef void
    (sig_cb_Hesai_Pandar_scan_point_cloud_xyz) (const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ> >&,
                                                float,
                                                float);
    /** \brief Signal used for a single sector
     *         Represents 1 corrected packet from the Pandar Hesai.  Each laser has a different RGB
     */
    typedef void
    (sig_cb_Hesai_Pandar_scan_point_cloud_xyzrgb) (const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGBA> >&,
                                                   float,
                                                   float);
    /** \brief Signal used for a single sector
     *         Represents 1 corrected packet from the Pandar Hesai with the returned intensity.
     */
    typedef void
    (sig_cb_Hesai_Pandar_scan_point_cloud_xyzi) (const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZI> >&,
                                                 float startAngle,
                                                 float);
    /** \brief Signal used for a 360 degree sweep
     *         Represents multiple corrected packets from the Pandar Hesai
     *         This signal is sent when the Hesai passes angle "0"
     */
    typedef void
    (sig_cb_Hesai_Pandar_sweep_point_cloud_xyz) (const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ> >&);
    /** \brief Signal used for a 360 degree sweep
     *         Represents multiple corrected packets from the Pandar Hesai with the returned intensity
     *         This signal is sent when the Hesai passes angle "0"
     */
    typedef void
    (sig_cb_Hesai_Pandar_sweep_point_cloud_xyzi) (const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZI> >&);
    /** \brief Signal used for a 360 degree sweep
     *         Represents multiple corrected packets from the Pandar Hesai
     *         This signal is sent when the Hesai passes angle "0".  Each laser has a different RGB
     */
    typedef void
    (sig_cb_Hesai_Pandar_sweep_point_cloud_xyzrgb) (const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGBA> >&);

    /** \brief Constructor taking an optional path to an Pandar corrections file.  The Grabber will listen on the default IP/port for data packets [192.168.3.255/2368]
     * \param[in] correctionsFile Path to a file which contains the correction parameters for the Pandar.  This parameter is mandatory for the Pandar-64, optional for the Pandar-32
     * \param[in] pcapFile Path to a file which contains previously captured data packets.  This parameter is optional
     */
    PandarGrabber (const std::string& correctionsFile = "",
                const std::string& pcapFile = "");

    /** \brief Constructor taking a pecified IP/port and an optional path to an Pandar corrections file.
     * \param[in] ipAddress IP Address that should be used to listen for Pandar packets
     * \param[in] port UDP Port that should be used to listen for Pandar packets
     * \param[in] correctionsFile Path to a file which contains the correction parameters for the Pandar.  This field is mandatory for the Pandar-64, optional for the Pandar-32
     */
    PandarGrabber (const boost::asio::ip::address& ipAddress,
                const unsigned short port,
                const std::string& correctionsFile = "");

    /** \brief virtual Destructor inherited from the Grabber interface. It never throws. */
    virtual
    ~PandarGrabber () throw ();

    /** \brief Starts processing the Hesai packets, either from the network or PCAP file. */
    virtual void
    start ();

    /** \brief Stops processing the Hesai packets, either from the network or PCAP file */
    virtual void
    stop ();

    /** \brief Obtains the name of this I/O Grabber
     *  \return The name of the grabber
     */
    virtual std::string
    getName () const;

    /** \brief Check if the grabber is still running.
     *  \return TRUE if the grabber is running, FALSE otherwise
     */
    virtual bool
    isRunning () const;

    /** \brief Returns the number of frames per second.
     */
    virtual float
    getFramesPerSecond () const;

    /** \brief Allows one to filter packets based on the SOURCE IP address and PORT
     *         This can be used, for instance, if multiple Pandar LIDARs are on the same network
     */
    void
    filterPackets (const boost::asio::ip::address& ipAddress,
                   const unsigned short port = 443);

    /** \brief Convert raw data to point cloud
     */
    void raw2PointCloud(const hesai::Scan& scan, pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);

    /** \brief Allows one to customize the colors used for each of the lasers.
     */
    void
    setLaserColorRGB (const pcl::RGB& color,
                      unsigned int laserNumber);

    /** \brief Any returns from the Pandar with a distance less than this are discarded.
     *         This value is in meters
     *         Default: 0.0
     */
    void
    setMinimumDistanceThreshold (float & minThreshold);

    /** \brief Any returns from the Pandar with a distance greater than this are discarded.
     *         This value is in meters
     *         Default: 10000.0
     */
    void
    setMaximumDistanceThreshold (float & maxThreshold);

    /** \brief Returns the current minimum distance threshold, in meters
     */

    float
    getMinimumDistanceThreshold ();

    /** \brief Returns the current maximum distance threshold, in meters
     */
    float
    getMaximumDistanceThreshold ();

  protected:
    static const int Pandar_DATA_PORT = 8080;
    static const int Pandar_NUM_ROT_ANGLES = 36001;
    static const int Pandar_LASER_PER_FIRING = HS_LIDAR_L40_UNIT_NUM;
    static const int Pandar_MAX_NUM_LASERS = HS_LIDAR_L40_UNIT_NUM;
    static const int Pandar_FIRING_PER_PKT = HS_LIDAR_L40_BLOCK_NUM;

    enum PandarBlock
    {
      BLOCK_0_TO_31 = 0xeeff, BLOCK_32_TO_63 = 0xddff
    };

#pragma pack(push, 1)
    typedef struct PandarLaserReturn
    {
        unsigned short distance;
        unsigned char intensity;
    } PandarLaserReturn;
#pragma pack(pop)

    struct PandarFiringData
    {
        unsigned short blockIdentifier;
        unsigned short rotationalPosition;
        PandarLaserReturn laserReturns[Pandar_LASER_PER_FIRING];
    };

    struct PandarDataPacket
    {
        PandarFiringData firingData[Pandar_FIRING_PER_PKT];
        unsigned int gpsTimestamp;
        unsigned char mode;
        unsigned char sensorType;
    };

    struct PandarLaserCorrection
    {
        double azimuthCorrection;
        double verticalCorrection;
        double distanceCorrection;
        double verticalOffsetCorrection;
        double horizontalOffsetCorrection;
        double sinVertCorrection;
        double cosVertCorrection;
        double sinVertOffsetCorrection;
        double cosVertOffsetCorrection;
    };




    PandarLaserCorrection laser_corrections_[Pandar_MAX_NUM_LASERS];
    unsigned int last_azimuth_;
    boost::shared_ptr<hesai::Scan> current_scan_raw_;
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > current_scan_xyz_, current_sweep_xyz_;
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI> > current_scan_xyzi_, current_sweep_xyzi_;
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA> > current_scan_xyzrgb_, current_sweep_xyzrgb_;
    boost::signals2::signal<sig_cb_Hesai_Pandar_sweep_point_cloud_xyz>* sweep_xyz_signal_;

    boost::signals2::signal<sig_cb_Hesai_Pandar_sweep_point_cloud_xyzrgb>* sweep_xyzrgb_signal_;
    boost::signals2::signal<sig_cb_Hesai_Pandar_sweep_point_cloud_xyzi>* sweep_xyzi_signal_;
    boost::signals2::signal<sig_cb_Hesai_Pandar_scan_raw_data>* scan_raw_signal_;
    boost::signals2::signal<sig_cb_Hesai_Pandar_scan_point_cloud_xyz>* scan_xyz_signal_;
    boost::signals2::signal<sig_cb_Hesai_Pandar_scan_point_cloud_xyzrgb>* scan_xyzrgb_signal_;
    boost::signals2::signal<sig_cb_Hesai_Pandar_scan_point_cloud_xyzi>* scan_xyzi_signal_;

    void
    fireCurrentSweep ();

    void
    fireCurrentScan (const unsigned short startAngle,
                     const unsigned short endAngle);
    void
    computeXYZI (pcl::PointXYZI& pointXYZI,
                 int azimuth,
                 HS_LIDAR_L40_Unit laserReturn,
                 PandarLaserCorrection correction);

    void
    computeXYZIfromRaw (pcl::PointXYZI& point,
                              const hesai::Scan_Measure& meas,
                              PandarLaserCorrection correction);


  private:
    hesai::Scan_Sweep* record_sweeps[Pandar_MAX_NUM_LASERS];

    static double *cos_lookup_table_;
    static double *sin_lookup_table_;
    pcl::SynchronizedQueue<unsigned char *> Pandar_data_;
    boost::asio::ip::udp::endpoint udp_listener_endpoint_;
    boost::asio::ip::address source_address_filter_;
    unsigned short source_port_filter_;
    boost::asio::io_service Pandar_read_socket_service_;
    boost::asio::ip::udp::socket *Pandar_read_socket_;
    std::string pcap_file_name_;
    boost::thread *queue_consumer_thread_;
    boost::thread *Pandar_read_packet_thread_;
    bool terminate_read_packet_thread_;
    pcl::RGB laser_rgb_mapping_[Pandar_MAX_NUM_LASERS];
    float min_distance_threshold_;
    float max_distance_threshold_;

    virtual void
    toPointClouds (HS_LIDAR_L40_Packet *dataPacket);

    virtual boost::asio::ip::address
    getDefaultNetworkAddress ();

    void
    initialize (const std::string& correctionsFile = "");

    void
    processHesaiPackets ();

    void
    enqueuePandarPacket (const unsigned char *data,
                      std::size_t bytesReceived);

    void
    loadCorrectionsFile (const std::string& correctionsFile);

    void
    loadPandar40Corrections ();

    void
    readPacketsFromSocket ();

    void
    readPacketsFromPcap();


    bool
    isAddressUnspecified (const boost::asio::ip::address& ip_address);

};
}
#endif /* PCL_IO_Pandar_GRABBER_H_ */
