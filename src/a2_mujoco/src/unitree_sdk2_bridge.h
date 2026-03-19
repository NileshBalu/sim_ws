#pragma once

#include <mujoco/mujoco.h>

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/dds_wrapper/robots/go2/go2.h>
#include <unitree/dds_wrapper/robots/g1/g1.h>
#include <unitree/idl/hg/BmsState_.hpp>
#include <unitree/idl/hg/IMUState_.hpp>

#include <iostream>
#include <cmath>
#include <vector>
#include <cstring>
#include <limits>

#include "param.h"
#include "physics_joystick.h"

#define MOTOR_SENSOR_NUM 3

class UnitreeSDK2BridgeBase
{
public:
    UnitreeSDK2BridgeBase(mjModel *model, mjData *data, std::recursive_mutex* sim_mtx)
    : mj_model_(model), mj_data_(data), sim_mtx_(sim_mtx)
    {
        _check_sensor();
        if(param::config.print_scene_information == 1) {
            printSceneInformation();
        }
        if(param::config.use_joystick == 1) {
            if(param::config.joystick_type == "xbox") {
                joystick = std::make_shared<XBoxJoystick>(param::config.joystick_device, param::config.joystick_bits);
            } else if(param::config.joystick_type == "switch") {
                joystick = std::make_shared<SwitchJoystick>(param::config.joystick_device, param::config.joystick_bits);
            } else {
                std::cerr << "Unsupported joystick type: " << param::config.joystick_type << std::endl;
                exit(EXIT_FAILURE);
            }
        }

        // Initialize Lidar Raycasting Buffers
        struct { double elev; double az_offset; } channel_calib_[128] = {
            { -43.4070058 * M_PI/180.0,  -1.075909345 * M_PI/180.0 },  // ch 1
            {  -3.500272004 * M_PI/180.0,  -9.713067778 * M_PI/180.0 },  // ch 2
            {  -1.842883942 * M_PI/180.0,  -1.089488685 * M_PI/180.0 },  // ch 3
            {  -1.645994908 * M_PI/180.0,  -1.657247245 * M_PI/180.0 },  // ch 4
            {  -0.748742608 * M_PI/180.0,  -1.103776484 * M_PI/180.0 },  // ch 5
            {   0.12920992  * M_PI/180.0,  -0.716998357 * M_PI/180.0 },  // ch 6
            {   0.992552154 * M_PI/180.0,  -1.118190147 * M_PI/180.0 },  // ch 7
            {   1.846800112 * M_PI/180.0,  -0.733952833 * M_PI/180.0 },  // ch 8
            {   2.640679567 * M_PI/180.0,   2.692378861 * M_PI/180.0 },  // ch 9
            {   3.461480498 * M_PI/180.0,   1.333121281 * M_PI/180.0 },  // ch 10
            {   4.293780093 * M_PI/180.0,   1.137149004 * M_PI/180.0 },  // ch 11
            {   5.092526983 * M_PI/180.0,   1.13414179  * M_PI/180.0 },  // ch 12
            {   5.908636069 * M_PI/180.0,   2.678227884 * M_PI/180.0 },  // ch 13
            {   6.687539148 * M_PI/180.0,   1.36124364  * M_PI/180.0 },  // ch 14
            {   7.487248042 * M_PI/180.0,   2.675511286 * M_PI/180.0 },  // ch 15
            {   8.248775296 * M_PI/180.0,   1.39049176  * M_PI/180.0 },  // ch 16
            {   9.076985132 * M_PI/180.0,  -1.190423879 * M_PI/180.0 },  // ch 17
            {   9.838375723 * M_PI/180.0,  -9.918608199 * M_PI/180.0 },  // ch 18
            {  10.59151018 * M_PI/180.0,  -9.815865003 * M_PI/180.0 },  // ch 19
            {  11.33910396 * M_PI/180.0,  -9.976507394 * M_PI/180.0 },  // ch 20
            {  12.07805231 * M_PI/180.0,  -1.221071601 * M_PI/180.0 },  // ch 21
            {  12.81269597 * M_PI/180.0, -10.04153818  * M_PI/180.0 },  // ch 22
            {  13.53925565 * M_PI/180.0,  -1.237135077 * M_PI/180.0 },  // ch 23
            {  14.26249607 * M_PI/180.0, -10.11374257  * M_PI/180.0 },  // ch 24
            {  15.1113837  * M_PI/180.0,  -1.496032197 * M_PI/180.0 },  // ch 25
            {  15.82565487 * M_PI/180.0,  -1.423791982 * M_PI/180.0 },  // ch 26
            {  16.52778989 * M_PI/180.0,   2.507271674 * M_PI/180.0 },  // ch 27
            {  17.23046308 * M_PI/180.0,   1.1557618   * M_PI/180.0 },  // ch 28
            {  17.92265498 * M_PI/180.0,   2.520626156 * M_PI/180.0 },  // ch 29
            {  18.61808487 * M_PI/180.0,   1.64916264  * M_PI/180.0 },  // ch 30
            {  19.31041248 * M_PI/180.0,   2.53605918  * M_PI/180.0 },  // ch 31
            {  19.99050556 * M_PI/180.0,   1.74981178  * M_PI/180.0 },  // ch 32
            {  20.67844375 * M_PI/180.0,  -1.329212174 * M_PI/180.0 },  // ch 33
            {  21.38383549 * M_PI/180.0, -10.59279064  * M_PI/180.0 },  // ch 34
            {  22.07211824 * M_PI/180.0,  -1.349371671 * M_PI/180.0 },  // ch 35
            {  22.73198747 * M_PI/180.0, -10.7135115   * M_PI/180.0 },  // ch 36
            {  23.42065969 * M_PI/180.0,  -1.370300438 * M_PI/180.0 },  // ch 37
            {  24.0699123  * M_PI/180.0, -10.8399154   * M_PI/180.0 },  // ch 38
            {  24.76068099 * M_PI/180.0,  -1.392023174 * M_PI/180.0 },  // ch 39
            {  26.06032489 * M_PI/180.0,  -1.392023174 * M_PI/180.0 },  // ch 40 (approx, verify)
            {  26.7342556  * M_PI/180.0,   3.9116336   * M_PI/180.0 },  // ch 41
            {  27.48752447 * M_PI/180.0,   2.672054604 * M_PI/180.0 },  // ch 42
            {  28.04757164 * M_PI/180.0,  14.56235895  * M_PI/180.0 },  // ch 43
            {  28.70785353 * M_PI/180.0,   2.702215683 * M_PI/180.0 },  // ch 44
            {  29.35493141 * M_PI/180.0,  12.71120938  * M_PI/180.0 },  // ch 45
            {  30.02433096 * M_PI/180.0,   2.734684653 * M_PI/180.0 },  // ch 46
            {  30.65706162 * M_PI/180.0,  12.38824562  * M_PI/180.0 },  // ch 47
            {  31.36997281 * M_PI/180.0,  -1.513633364 * M_PI/180.0 },  // ch 48
            {  32.04273103 * M_PI/180.0,  -1.768390723 * M_PI/180.0 },  // ch 49
            {  32.67908352 * M_PI/180.0,  -1.540843922 * M_PI/180.0 },  // ch 50
            {  33.34105078 * M_PI/180.0, -11.95235285  * M_PI/180.0 },  // ch 51
            {  33.98525473 * M_PI/180.0,  -1.569145141 * M_PI/180.0 },  // ch 52
            {  34.63508059 * M_PI/180.0, -12.08126      * M_PI/180.0 },  // ch 53
            {  35.28890657 * M_PI/180.0,  -1.598607805 * M_PI/180.0 },  // ch 54
            {  35.91924152 * M_PI/180.0, -12.35419678  * M_PI/180.0 },  // ch 55
            {  36.56991405 * M_PI/180.0,   1.3980024   * M_PI/180.0 },  // ch 56
            {  37.23291572 * M_PI/180.0,  13.96002472  * M_PI/180.0 },  // ch 57
            {  37.88520899 * M_PI/180.0,   2.98490228  * M_PI/180.0 },  // ch 58
            {  38.51337468 * M_PI/180.0,  14.21126251  * M_PI/180.0 },  // ch 59
            {  39.15446038 * M_PI/180.0,   3.037441702 * M_PI/180.0 },  // ch 60
            {  39.79145356 * M_PI/180.0,  14.47804807  * M_PI/180.0 },  // ch 61
            {  40.45130698 * M_PI/180.0,   3.072447285 * M_PI/180.0 },  // ch 62
            {  41.06843677 * M_PI/180.0,  14.76155709  * M_PI/180.0 },  // ch 63
            {  41.78202703 * M_PI/180.0,  -1.766483059 * M_PI/180.0 },  // ch 64
            {  42.44706991 * M_PI/180.0, -13.6157838   * M_PI/180.0 },  // ch 65
            {  43.07798426 * M_PI/180.0,  -1.804947337 * M_PI/180.0 },  // ch 66
            {  43.69953423 * M_PI/180.0, -13.81354396  * M_PI/180.0 },  // ch 67
            {  44.37382533 * M_PI/180.0,  -1.845357894 * M_PI/180.0 },  // ch 68
            {  45.01872602 * M_PI/180.0, -14.21918905  * M_PI/180.0 },  // ch 69
            {  45.69886152 * M_PI/180.0,  -1.886224892 * M_PI/180.0 },  // ch 70
            {  46.29110525 * M_PI/180.0, -14.54995653  * M_PI/180.0 },  // ch 71
            {  47.64591734 * M_PI/180.0,   3.442232655 * M_PI/180.0 },  // ch 72
            {  47.64591734 * M_PI/180.0,  15.35398958  * M_PI/180.0 },  // ch 73 (approx)
            {  48.91294405 * M_PI/180.0,   3.521484027 * M_PI/180.0 },  // ch 74 (approx)
            {  49.48291634 * M_PI/180.0,  15.66316927  * M_PI/180.0 },  // ch 75 (approx)
            {  50.17996405 * M_PI/180.0,   3.6219693   * M_PI/180.0 },  // ch 76 (approx)
            {  50.82483575 * M_PI/180.0,   3.721893969 * M_PI/180.0 },  // ch 77 (approx)
            {  51.16513155 * M_PI/180.0, -13.39550201  * M_PI/180.0 },  // ch 78 (approx)
            {  52.16513155 * M_PI/180.0, -13.39550201  * M_PI/180.0 },  // ch 79 (approx, verify)
            {  53.46964786 * M_PI/180.0, -16.65208595  * M_PI/180.0 },  // ch 80 (approx)
            {  54.77660254 * M_PI/180.0, -17.15726363  * M_PI/180.0 },  // ch 81 (approx)
            {  55.38480725 * M_PI/180.0, -17.30324269  * M_PI/180.0 },  // ch 82 (approx)
            {  56.65624596 * M_PI/180.0, -18.2951064   * M_PI/180.0 },  // ch 83 (approx)
            {  57.35379221 * M_PI/180.0,   4.371527296 * M_PI/180.0 },  // ch 84 (approx)
            {  58.10005594 * M_PI/180.0,  21.24102563  * M_PI/180.0 },  // ch 85 (approx)
            {  58.67531241 * M_PI/180.0,   4.5460923   * M_PI/180.0 },  // ch 86 (approx)
            {  59.37996238 * M_PI/180.0,  22.05194779  * M_PI/180.0 },  // ch 87 (approx)
            {  60.06321171 * M_PI/180.0,   4.727221604 * M_PI/180.0 },  // ch 88 (approx)
            {  61.32558317 * M_PI/180.0,  23.95619095  * M_PI/180.0 },  // ch 89 (approx)
            {  62.07259545 * M_PI/180.0,  -2.7703891   * M_PI/180.0 },  // ch 90 (approx)
            {  63.43501469 * M_PI/180.0, -22.51096069  * M_PI/180.0 },  // ch 91 (approx)
            {  64.05871771 * M_PI/180.0,  -2.884330945 * M_PI/180.0 },  // ch 92 (approx)
            {  65.00093066 * M_PI/180.0, -24.82582478  * M_PI/180.0 },  // ch 93 (approx)
            {  66.78254105 * M_PI/180.0, -26.20398136  * M_PI/180.0 },  // ch 94 (approx)
            {  67.28622222 * M_PI/180.0, -26.20398136  * M_PI/180.0 },  // ch 95 (approx, verify)
            {  69.49486092 * M_PI/180.0,  63.41592976  * M_PI/180.0 },  // ch 96 (approx)
            {  69.12016034 * M_PI/180.0,  32.51541429  * M_PI/180.0 },  // ch 97 (approx)
            {  69.48628049 * M_PI/180.0,  34.7889226   * M_PI/180.0 },  // ch 98 (approx)
            {  70.36538155 * M_PI/180.0,  34.7889226   * M_PI/180.0 },  // ch 99 (approx, verify)
            {  70.89276896 * M_PI/180.0,   7.394151622 * M_PI/180.0 },  // ch 100 (approx)
            {  71.34965096 * M_PI/180.0,  40.42367283  * M_PI/180.0 },  // ch 101 (approx)
            {  72.31773888 * M_PI/180.0,   8.113894071 * M_PI/180.0 },  // ch 102 (approx)
            {  72.76991345 * M_PI/180.0,  10.46458828  * M_PI/180.0 },  // ch 103 (approx)
            {  73.87760985 * M_PI/180.0,  -4.247621383 * M_PI/180.0 },  // ch 104 (approx)
            {  75.22200785 * M_PI/180.0,  10.40492055  * M_PI/180.0 },  // ch 105 (approx)
            {  75.35638155 * M_PI/180.0,  -5.044677435 * M_PI/180.0 },  // ch 106 (approx)
            {  76.00001128 * M_PI/180.0, -45.72193115  * M_PI/180.0 },  // ch 107 (approx)
            {  76.84965873 * M_PI/180.0,  -5.044677435 * M_PI/180.0 },  // ch 108 (approx, verify)
            {  77.50845696 * M_PI/180.0, -50.94643093  * M_PI/180.0 },  // ch 109 (approx)
            {  78.15388776 * M_PI/180.0, -52.27909987  * M_PI/180.0 },  // ch 110 (approx)
            {  79.02224323 * M_PI/180.0,  26.63341482  * M_PI/180.0 },  // ch 111 (approx)
            {  81.04256975 * M_PI/180.0,  -6.320073689 * M_PI/180.0 },  // ch 112 (approx)
            {  82.40442354 * M_PI/180.0,  38.1489664   * M_PI/180.0 },  // ch 113 (approx)
            {  83.43592513 * M_PI/180.0,  49.40469446  * M_PI/180.0 },  // ch 114 (approx)
            {  83.5792221  * M_PI/180.0,  -6.320073689 * M_PI/180.0 },  // ch 115 (approx, verify)
            {  86.40880798 * M_PI/180.0,  18.70038383  * M_PI/180.0 },  // ch 116 (approx)
            {  88.9090967  * M_PI/180.0, 117.3035311   * M_PI/180.0 },  // ch 117 (approx)
        };
        
        nray_ = h_samples_ * v_samples_;
        ray_vecs_.resize(nray_ * 3);
        local_ray_vecs_.resize(nray_ * 3);
        ray_geomid_.resize(nray_);
        ray_dist_.resize(nray_);
        ray_normal_.resize(nray_ * 3);
        
        double theta_step = (h_samples_ > 1)
            ? (max_theta_ - min_theta_) / (h_samples_ - 1)
            : 0.0;
        
        for (int i = 0; i < h_samples_; ++i) {
            double theta = min_theta_ + i * theta_step;
        
            for (int j = 0; j < v_samples_; ++j) {
                double phi              = channel_calib_[j].elev;
                double theta_corrected  = theta + channel_calib_[j].az_offset;
        
                int idx = j * h_samples_ + i;
                local_ray_vecs_[3 * idx + 0] = std::cos(theta_corrected) * std::cos(phi);
                local_ray_vecs_[3 * idx + 1] = std::sin(theta_corrected) * std::cos(phi);
                local_ray_vecs_[3 * idx + 2] = std::sin(phi);
            }
        }
        
    }

    virtual void start() {}

    void printSceneInformation()
    {
        auto printObjects = [this](const char* title, int count, int type, auto getIndex) {
            std::cout << "<<------------- " << title << " ------------->> " << std::endl;
            for (int i = 0; i < count; i++) {
                const char* name = mj_id2name(mj_model_, type, i);
                if (name) {
                    std::cout << title << "_index: " << getIndex(i) << ", " << "name: " << name;
                    if (type == mjOBJ_SENSOR) {
                        std::cout << ", dim: " << mj_model_->sensor_dim[i];
                    }
                    std::cout << std::endl;
                }
            }
            std::cout << std::endl;
        };

        printObjects("Link", mj_model_->nbody, mjOBJ_BODY, [](int i) { return i; });
        printObjects("Joint", mj_model_->njnt, mjOBJ_JOINT, [](int i) { return i; });
        printObjects("Actuator", mj_model_->nu, mjOBJ_ACTUATOR, [](int i) { return i; });

        int sensorIndex = 0;
        printObjects("Sensor", mj_model_->nsensor, mjOBJ_SENSOR, [&](int i) {
            int currentIndex = sensorIndex;
            sensorIndex += mj_model_->sensor_dim[i];
            return currentIndex;
        });
    }

protected:
    int num_motor_ = 0;
    int dim_motor_sensor_ = 0;

    mjData *mj_data_;
    mjModel *mj_model_;

    int imu_quat_adr_ = -1;
    int imu_gyro_adr_ = -1;
    int imu_acc_adr_ = -1;
    int frame_pos_adr_ = -1;
    int frame_vel_adr_ = -1;

    int secondary_imu_quat_adr_ = -1;
    int secondary_imu_gyro_adr_ = -1;
    int secondary_imu_acc_adr_ = -1;

    std::shared_ptr<unitree::common::UnitreeJoystick> joystick = nullptr;

    std::recursive_mutex* sim_mtx_;

    std::shared_ptr<unitree::robot::ChannelPublisher<sensor_msgs::msg::dds_::PointCloud2_>> lidar_publisher_;

    // Lidar buffers
    int h_samples_ = 360;
    int v_samples_ = 128;
    int nray_ = h_samples_ * v_samples_;
    mjtNum cutoff_ = 10.0;
    double min_theta_ = -M_PI;
    double max_theta_ = M_PI;
    double min_phi_ = -M_PI * 4.5 / 180.0;
    double max_phi_ = M_PI / 3.0;

    std::vector<mjtNum> local_ray_vecs_;
    std::vector<mjtNum> ray_vecs_;
    std::vector<int> ray_geomid_;
    std::vector<mjtNum> ray_dist_;
    std::vector<mjtNum> ray_normal_;

    double lidar_publish_rate_ = 40.0;
    double next_lidar_time_ = 0.0;

    void _check_sensor()
    {
        num_motor_ = mj_model_->nu;
        dim_motor_sensor_ = MOTOR_SENSOR_NUM * num_motor_;

        int sensor_id = -1;

        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "imu_quat");
        if (sensor_id >= 0) imu_quat_adr_ = mj_model_->sensor_adr[sensor_id];

        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "imu_gyro");
        if (sensor_id >= 0) imu_gyro_adr_ = mj_model_->sensor_adr[sensor_id];

        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "imu_acc");
        if (sensor_id >= 0) imu_acc_adr_ = mj_model_->sensor_adr[sensor_id];

        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "frame_pos");
        if (sensor_id >= 0) frame_pos_adr_ = mj_model_->sensor_adr[sensor_id];

        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "frame_vel");
        if (sensor_id >= 0) frame_vel_adr_ = mj_model_->sensor_adr[sensor_id];

        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "secondary_imu_quat");
        if (sensor_id >= 0) secondary_imu_quat_adr_ = mj_model_->sensor_adr[sensor_id];

        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "secondary_imu_gyro");
        if (sensor_id >= 0) secondary_imu_gyro_adr_ = mj_model_->sensor_adr[sensor_id];

        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "secondary_imu_acc");
        if (sensor_id >= 0) secondary_imu_acc_adr_ = mj_model_->sensor_adr[sensor_id];
    }
};

template <typename LowCmd_t, typename LowState_t>
class RobotBridge : public UnitreeSDK2BridgeBase
{
using HighState_t = unitree::robot::go2::publisher::SportModeState;
using WirelessController_t = unitree::robot::go2::publisher::WirelessController;

public:
    RobotBridge(mjModel *model, mjData *data, std::recursive_mutex* sim_mtx)
    : UnitreeSDK2BridgeBase(model, data, sim_mtx)
    {
        lowcmd = std::make_shared<LowCmd_t>("rt/lowcmd");
        lowstate = std::make_unique<LowState_t>();
        lowstate->joystick = joystick;
        highstate = std::make_unique<HighState_t>();
        wireless_controller = std::make_unique<WirelessController_t>();
        wireless_controller->joystick = joystick;

        lidar_publisher_ = std::make_shared<unitree::robot::ChannelPublisher<sensor_msgs::msg::dds_::PointCloud2_>>("rt/mujoco/lidar");
        lidar_publisher_->InitChannel();
    }

    void start()
    {
        thread_ = std::make_shared<unitree::common::RecurrentThread>(
            "unitree_bridge", UT_CPU_ID_NONE, 1000, [this]() { this->run(); });
    }

    virtual void run()
    {
        if(!mj_data_) return;
        if(lowstate->joystick) { lowstate->joystick->update(); }

        std::lock_guard<std::recursive_mutex> sim_lock(*sim_mtx_);

        // lowcmd
        {
            std::lock_guard<std::mutex> lock(lowcmd->mutex_);
            for(int i(0); i < num_motor_; i++) {
                auto & m = lowcmd->msg_.motor_cmd()[i];
                mj_data_->ctrl[i] = m.tau() +
                                    m.kp() * (m.q() - mj_data_->sensordata[i]) +
                                    m.kd() * (m.dq() - mj_data_->sensordata[i + num_motor_]);
            }
        }

        // lowstate
        if(lowstate->trylock()) {
            for(int i(0); i < num_motor_; i++) {
                lowstate->msg_.motor_state()[i].q()       = mj_data_->sensordata[i];
                lowstate->msg_.motor_state()[i].dq()      = mj_data_->sensordata[i + num_motor_];
                lowstate->msg_.motor_state()[i].tau_est() = mj_data_->sensordata[i + 2 * num_motor_];
            }

            if(imu_quat_adr_ >= 0) {
                lowstate->msg_.imu_state().quaternion()[0] = mj_data_->sensordata[imu_quat_adr_ + 0];
                lowstate->msg_.imu_state().quaternion()[1] = mj_data_->sensordata[imu_quat_adr_ + 1];
                lowstate->msg_.imu_state().quaternion()[2] = mj_data_->sensordata[imu_quat_adr_ + 2];
                lowstate->msg_.imu_state().quaternion()[3] = mj_data_->sensordata[imu_quat_adr_ + 3];

                double w = lowstate->msg_.imu_state().quaternion()[0];
                double x = lowstate->msg_.imu_state().quaternion()[1];
                double y = lowstate->msg_.imu_state().quaternion()[2];
                double z = lowstate->msg_.imu_state().quaternion()[3];

                lowstate->msg_.imu_state().rpy()[0] = atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y));
                lowstate->msg_.imu_state().rpy()[1] = asin(2*(w*y - z*x));
                lowstate->msg_.imu_state().rpy()[2] = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z));
            }

            if(imu_gyro_adr_ >= 0) {
                lowstate->msg_.imu_state().gyroscope()[0] = mj_data_->sensordata[imu_gyro_adr_ + 0];
                lowstate->msg_.imu_state().gyroscope()[1] = mj_data_->sensordata[imu_gyro_adr_ + 1];
                lowstate->msg_.imu_state().gyroscope()[2] = mj_data_->sensordata[imu_gyro_adr_ + 2];
            }

            if(imu_acc_adr_ >= 0) {
                lowstate->msg_.imu_state().accelerometer()[0] = mj_data_->sensordata[imu_acc_adr_ + 0];
                lowstate->msg_.imu_state().accelerometer()[1] = mj_data_->sensordata[imu_acc_adr_ + 1];
                lowstate->msg_.imu_state().accelerometer()[2] = mj_data_->sensordata[imu_acc_adr_ + 2];
            }

            lowstate->msg_.tick() = std::round(mj_data_->time / 1e-3);
            lowstate->unlockAndPublish();
        }

        // highstate
        if(highstate->trylock()) {
            if(frame_pos_adr_ >= 0) {
                highstate->msg_.position()[0] = mj_data_->sensordata[frame_pos_adr_ + 0];
                highstate->msg_.position()[1] = mj_data_->sensordata[frame_pos_adr_ + 1];
                highstate->msg_.position()[2] = mj_data_->sensordata[frame_pos_adr_ + 2];
            }
            if(frame_vel_adr_ >= 0) {
                highstate->msg_.velocity()[0] = mj_data_->sensordata[frame_vel_adr_ + 0];
                highstate->msg_.velocity()[1] = mj_data_->sensordata[frame_vel_adr_ + 1];
                highstate->msg_.velocity()[2] = mj_data_->sensordata[frame_vel_adr_ + 2];
            }
            highstate->unlockAndPublish();
        }

        // wireless_controller
        if(wireless_controller->joystick) {
            wireless_controller->unlockAndPublish();
        }

        // ---- Lidar Raycasting ----
        if (mj_data_->time >= next_lidar_time_) {
            next_lidar_time_ = mj_data_->time + 1.0 / lidar_publish_rate_;

            int body_id = mj_name2id(mj_model_, mjOBJ_BODY, "torso_link");
            if (body_id < 0) {
                body_id = mj_name2id(mj_model_, mjOBJ_BODY, "base_link");
            }

            if (body_id >= 0) {
                mjtNum pnt[3] = {
                    mj_data_->xpos[3 * body_id + 0],
                    mj_data_->xpos[3 * body_id + 1],
                    mj_data_->xpos[3 * body_id + 2] + 0.2
                };

                // Rotate local ray directions to global for mj_multiRay
                mjtNum* xmat = mj_data_->xmat + 9 * body_id;
                for (int i = 0; i < nray_; i++) {
                    ray_vecs_[3 * i + 0] = xmat[0] * local_ray_vecs_[3 * i + 0] + xmat[1] * local_ray_vecs_[3 * i + 1] + xmat[2] * local_ray_vecs_[3 * i + 2];
                    ray_vecs_[3 * i + 1] = xmat[3] * local_ray_vecs_[3 * i + 0] + xmat[4] * local_ray_vecs_[3 * i + 1] + xmat[5] * local_ray_vecs_[3 * i + 2];
                    ray_vecs_[3 * i + 2] = xmat[6] * local_ray_vecs_[3 * i + 0] + xmat[7] * local_ray_vecs_[3 * i + 1] + xmat[8] * local_ray_vecs_[3 * i + 2];
                }

                mj_multiRay(mj_model_, mj_data_, pnt, ray_vecs_.data(), nullptr, 1, body_id,
                            ray_geomid_.data(), ray_dist_.data(), ray_normal_.data(), nray_, cutoff_);

                if (lidar_publisher_) {
                    sensor_msgs::msg::dds_::PointCloud2_ msg;

                    msg.header().frame_id() = "lidar_link";
                    msg.header().stamp().sec()     = static_cast<int32_t>(mj_data_->time);
                    msg.header().stamp().nanosec() = static_cast<uint32_t>(
                        (mj_data_->time - msg.header().stamp().sec()) * 1e9);

                    msg.height()       = 1;
                    msg.width()        = nray_;
                    msg.is_dense()     = false;
                    msg.is_bigendian() = false;

                    msg.fields().resize(7);
                    auto set_field = [&](int idx, const std::string& name,
                                        uint32_t offset, uint8_t datatype) {
                        msg.fields()[idx].name()     = name;
                        msg.fields()[idx].offset()   = offset;
                        msg.fields()[idx].datatype() = datatype;
                        msg.fields()[idx].count()    = 1;
                    };
                    set_field(0, "x",       0,  7); // FLOAT32
                    set_field(1, "y",       4,  7);
                    set_field(2, "z",       8,  7);
                    set_field(3, "dist",   12,  7);
                    set_field(4, "nx",     16,  7);
                    set_field(5, "ny",     20,  7);
                    set_field(6, "nz",     24,  7);

                    msg.point_step() = 28;
                    msg.row_step()   = nray_ * msg.point_step();
                    msg.data().resize(msg.row_step());

                    float max_distance = 10.0f;
                    uint8_t* ptr = msg.data().data();
                    for (int i = 0; i < nray_; i++) {
                        float   dist   = static_cast<float>(ray_dist_[i]);
                        if (dist == -1.0f || dist > max_distance) {
                            dist = max_distance;
                        }
                        // std::cout<<"dist: "<<dist<<std::endl;
                        float   vx     = static_cast<float>(local_ray_vecs_[i*3+0]);
                        float   vy     = static_cast<float>(local_ray_vecs_[i*3+1]);
                        float   vz     = static_cast<float>(local_ray_vecs_[i*3+2]);
                        float   nx     = static_cast<float>(ray_normal_[i*3+0]);
                        float   ny     = static_cast<float>(ray_normal_[i*3+1]);
                        float   nz     = static_cast<float>(ray_normal_[i*3+2]);

                        float x = dist * vx;
                        float y = dist * vy;
                        float z = dist * vz;

                        std::memcpy(ptr,      &x,      4);
                        std::memcpy(ptr +  4, &y,      4);
                        std::memcpy(ptr +  8, &z,      4);
                        std::memcpy(ptr + 12, &dist,   4);
                        std::memcpy(ptr + 16, &nx,     4);
                        std::memcpy(ptr + 20, &ny,     4);
                        std::memcpy(ptr + 24, &nz,     4);
                        ptr += 28;
                    }

                    lidar_publisher_->Write(msg);
                }
            }
        }
    }

    std::unique_ptr<HighState_t> highstate;
    std::unique_ptr<WirelessController_t> wireless_controller;
    std::shared_ptr<LowCmd_t> lowcmd;
    std::unique_ptr<LowState_t> lowstate;

private:
    unitree::common::RecurrentThreadPtr thread_;
};

using Go2Bridge = RobotBridge<unitree::robot::go2::subscription::LowCmd, unitree::robot::go2::publisher::LowState>;

class G1Bridge : public RobotBridge<unitree::robot::g1::subscription::LowCmd, unitree::robot::g1::publisher::LowState>
{
public:
    G1Bridge(mjModel *model, mjData *data, std::recursive_mutex* sim_mtx)
    : RobotBridge(model, data, sim_mtx)
    {
        if (param::config.robot.find("g1") != std::string::npos) {
            auto* g1_lowstate = dynamic_cast<unitree::robot::g1::publisher::LowState*>(lowstate.get());
            if (g1_lowstate) {
                auto scene = param::config.robot_scene.filename().string();
                g1_lowstate->msg_.mode_machine() = scene.find("23") != std::string::npos ? 4 : 5;
            }
        }

        bmsstate = std::make_unique<BmsState_t>("rt/lf/bmsstate");
        bmsstate->msg_.soc() = 100;

        secondary_imustate = std::make_unique<IMUState_t>("rt/secondary_imu");
    }

    void run() override
    {
        RobotBridge::run();

        if (secondary_imustate->trylock()) {
            std::lock_guard<std::recursive_mutex> sim_lock(*sim_mtx_);

            if(secondary_imu_quat_adr_ >= 0) {
                secondary_imustate->msg_.quaternion()[0] = mj_data_->sensordata[secondary_imu_quat_adr_ + 0];
                secondary_imustate->msg_.quaternion()[1] = mj_data_->sensordata[secondary_imu_quat_adr_ + 1];
                secondary_imustate->msg_.quaternion()[2] = mj_data_->sensordata[secondary_imu_quat_adr_ + 2];
                secondary_imustate->msg_.quaternion()[3] = mj_data_->sensordata[secondary_imu_quat_adr_ + 3];

                double w = secondary_imustate->msg_.quaternion()[0];
                double x = secondary_imustate->msg_.quaternion()[1];
                double y = secondary_imustate->msg_.quaternion()[2];
                double z = secondary_imustate->msg_.quaternion()[3];

                secondary_imustate->msg_.rpy()[0] = atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y));
                secondary_imustate->msg_.rpy()[1] = asin(2*(w*y - z*x));
                secondary_imustate->msg_.rpy()[2] = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z));
            }

            if(secondary_imu_gyro_adr_ >= 0) {
                secondary_imustate->msg_.gyroscope()[0] = mj_data_->sensordata[secondary_imu_gyro_adr_ + 0];
                secondary_imustate->msg_.gyroscope()[1] = mj_data_->sensordata[secondary_imu_gyro_adr_ + 1];
                secondary_imustate->msg_.gyroscope()[2] = mj_data_->sensordata[secondary_imu_gyro_adr_ + 2];
            }

            if(secondary_imu_acc_adr_ >= 0) {
                secondary_imustate->msg_.accelerometer()[0] = mj_data_->sensordata[secondary_imu_acc_adr_ + 0];
                secondary_imustate->msg_.accelerometer()[1] = mj_data_->sensordata[secondary_imu_acc_adr_ + 1];
                secondary_imustate->msg_.accelerometer()[2] = mj_data_->sensordata[secondary_imu_acc_adr_ + 2];
            }

            secondary_imustate->unlockAndPublish();
        }

        bmsstate->unlockAndPublish();
    }

    using BmsState_t = unitree::robot::RealTimePublisher<unitree_hg::msg::dds_::BmsState_>;
    using IMUState_t = unitree::robot::RealTimePublisher<unitree_hg::msg::dds_::IMUState_>;
    std::unique_ptr<BmsState_t> bmsstate;
    std::unique_ptr<IMUState_t> secondary_imustate;
};