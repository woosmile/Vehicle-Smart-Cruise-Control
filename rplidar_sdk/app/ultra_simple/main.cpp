/*
 *  SLAMTEC LIDAR
 *  Ultra Simple Data Grabber Demo App
 *
 *  Copyright (c) 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  Copyright (c) 2014 - 2020 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 *
 */
/*
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>

#include "sl_lidar.h" 
#include "sl_lidar_driver.h"
#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#ifdef _WIN32
#include <Windows.h>
#define delay(x)   ::Sleep(x)
#else
#include <unistd.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include <thread>
#include <atomic>
#include <vector>
#include <opencv2/opencv.hpp>

#define EMPTY 0
#define FRONT 1
#define RIGHT 2
#define REAR 3
#define LEFT 4

#define DETECTION_THRESHOLD_MM 700
#define MAX_SPEED 150
#define NORMAL_SPEED 80

std::atomic<int> speed(80);
std::atomic<int> detected_direction(0);
std::atomic<bool> shock(false);
std::atomic<bool> ctrl_c_pressed(false);

template <typename T>
T clamp(T val, T low, T high) {
    if (val < low) return low;
    else if (val > high) return high;
    else return val;
}

static inline void delay(sl_word_size_t ms){
    while (ms>=1000){
        usleep(1000*1000);
        ms-=1000;
    };
    if (ms!=0)
        usleep(ms*1000);
}
#endif

using namespace sl;

void print_usage(int argc, const char * argv[])
{
    printf("Usage:\n"
           " For serial channel\n %s --channel --serial <com port> [baudrate]\n"
           " The baudrate used by different models is as follows:\n"
           "  A1(115200),A2M7(256000),A2M8(115200),A2M12(256000),"
           "A3(256000),S1(256000),S2(1000000),S3(1000000)\n"
		   " For udp channel\n %s --channel --udp <ipaddr> [port NO.]\n"
           " The T1 default ipaddr is 192.168.11.2,and the port NO.is 8089. Please refer to the datasheet for details.\n"
           , argv[0], argv[0]);
}

bool checkSLAMTECLIDARHealth(ILidarDriver * drv)
{
    sl_result     op_result;
    sl_lidar_response_device_health_t healthinfo;

    op_result = drv->getHealth(healthinfo);
    if (SL_IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
        printf("SLAMTEC Lidar health status : %d\n", healthinfo.status);
        if (healthinfo.status == SL_LIDAR_STATUS_ERROR) {
            fprintf(stderr, "Error, slamtec lidar internal error detected. Please reboot the device to retry.\n");
            // enable the following code if you want slamtec lidar to be reboot by software
            // drv->reset();
            return false;
        } else {
            return true;
        }

    } else {
        fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
        return false;
    }
}

void ctrlc(int)
{
    ctrl_c_pressed.store(true);
}

int canSetting(int *send_socket, int *receive_socket) {
    // CAN Variables
    int send_ret, receive_ret;
    struct sockaddr_can addr;
    struct ifreq ifr;

    // CAN Setting Start
    system("sudo ip link set can0 type can bitrate 500000");
    system("sudo ifconfig can0 up");
    // printf("this is a can send demo\r\n");
        
    //1.Create socket
    *send_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    *receive_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (*send_socket < 0 || *receive_socket < 0) {
        perror("socket PF_CAN failed");
        return -1;
    }
    
    //2.Specify can0 device
    strcpy(ifr.ifr_name, "can0");
    send_ret = ioctl(*send_socket, SIOCGIFINDEX, &ifr);
    receive_ret = ioctl(*receive_socket, SIOCGIFINDEX, &ifr);
    if (send_ret < 0 || receive_ret < 0) {
        perror("ioctl failed");
        return -1;
    }
    
    //3.Bind the socket to can0
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    send_ret = bind(*send_socket, (struct sockaddr *)&addr, sizeof(addr));
    receive_ret = bind(*receive_socket, (struct sockaddr *)&addr, sizeof(addr));
    if (send_ret < 0 || receive_ret < 0) {
        perror("bind failed");
        return -1;
    }

    //4.Disable filtering rules, do not receive packets, only send
    setsockopt(*send_socket, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);

    //4.Define receive rules
    struct can_filter rfilter[1];
    rfilter[0].can_id = 0x006;
    rfilter[0].can_mask = CAN_SFF_MASK;
    setsockopt(*receive_socket, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));
    
    return 0;
}

void scanLidar(ILidarDriver * drv, sl_result op_result, int s) {
    int nbytes;
    struct can_frame frame;
    memset(&frame, 0, sizeof(struct can_frame));

    //5.Set send data
    frame.can_id = 0x007;
    frame.can_dlc = 8;
    frame.data[0] = 0;
    frame.data[1] = 0;
    frame.data[2] = 0;
    frame.data[3] = 0;
    frame.data[4] = 0;
    frame.data[5] = 0;
    frame.data[6] = 0;
    frame.data[7] = 0;
    
    int prev_front_dist = -1;
    int prev_rear_dist = -1;

    drv->startScan(0,1);

    // fetech result and print it out...
    while (!ctrl_c_pressed.load()) {
        sl_lidar_response_measurement_node_hq_t nodes[8192];
        size_t count = _countof(nodes);

        op_result = drv->grabScanDataHq(nodes, count);

        if (SL_IS_OK(op_result)) {
            drv->ascendScanData(nodes, count);

            int detected_angle = -1;
            int min_dist = 2147483647;

            int cur_front_dist = 0;
            int cur_rear_dist = 0;

            for (int pos = 0; pos < (int)count; ++pos) {
                int angle = (nodes[pos].angle_z_q14 * 90.f) / 16384.f;
                int distance = nodes[pos].dist_mm_q2 / 4.0f;
                int q = nodes[pos].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT;

                if (distance < DETECTION_THRESHOLD_MM && distance > 0) {
                    if (angle < 0) angle += 360;
                    if (angle >= 360) angle -= 360;

                    if ((distance < min_dist) && (q > 0)) {
                        min_dist = distance;
                        detected_angle = angle;
                    }
                }
            }

            if ((detected_angle >= 315 && detected_angle <= 360) || (detected_angle >= 0 && detected_angle <= 45)) {
                cur_front_dist = min_dist;
                detected_direction = FRONT;  // 전면
            }
            else if (detected_angle > 45 && detected_angle <= 135) {
                detected_direction = RIGHT;  // 우측
            }
            else if (detected_angle > 135 && detected_angle <= 225) {
                cur_rear_dist = min_dist;
                detected_direction = REAR;  // 후면
            }
            else if (detected_angle > 225 && detected_angle < 315) {
                detected_direction = LEFT;  // 좌측
            }
            else {
                detected_direction = EMPTY;
            }

            if (detected_direction != FRONT && detected_direction != REAR) {
                if (speed > NORMAL_SPEED)
                    speed--;
                else if (speed < NORMAL_SPEED)
                    speed++;
            }

            if (prev_front_dist > 0 && cur_front_dist > 0) {
                if (cur_front_dist < prev_front_dist) {
                    // 전면 물체가 가까워지고 있음 → 감속
                    if (speed > 0)
                        speed--;
                }
                else {
                    if (speed > NORMAL_SPEED)
                        speed--;
                    else if (speed < NORMAL_SPEED)
                        speed++;
                }
            }

            if (prev_rear_dist > 0 && cur_rear_dist > 0) {
                if (cur_rear_dist < prev_rear_dist) {
                    // 후면 물체가 가까워지고 있음 → 가속
                    if (speed < MAX_SPEED)
                        speed++;
                }
                else {
                    if (speed > NORMAL_SPEED)
                        speed--;
                    else if (speed < NORMAL_SPEED)
                        speed++;
                }
            }
            
            // Detection 방향, Speed 값 CAN 통신
            frame.data[0] = detected_direction;
            frame.data[1] = speed;
            nbytes = write(s, &frame, sizeof(frame)); 
            if(nbytes != sizeof(frame)) {
                printf("Send Error frame!\r\n");
                system("sudo ifconfig can0 down");
                ctrl_c_pressed.store(true);
            }

            // 이전 거리 갱신
            prev_front_dist = cur_front_dist;
            prev_rear_dist = cur_rear_dist;

        }
        //std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
}

void onFinished(ILidarDriver * drv, int send_socket, int receive_socket) {
    if(drv) {
        delete drv;
        drv = NULL;
        close(send_socket);
        close(receive_socket);
        system("sudo ifconfig can0 down");
    }
}

std::vector<cv::Mat> loadSequence(const std::string& folder, int frameCount) {
    std::vector<cv::Mat> frames;
    for (int i = frameCount; i > 0; --i) {
        std::string filename = folder + "/background" + std::to_string(i) + ".png";
        cv::Mat frame = cv::imread(filename);
        if (!frame.empty()) {
            cv::resize(frame, frame, cv::Size(640, 360));
            // cv::resize(frame, frame, cv::Size(960, 480));
            frames.push_back(frame);
        }
    }
    return frames;
}

void playSequence(const std::vector<cv::Mat>& frames) {
    int frameIndex = 0;

    int min_speed = 30;
    int max_speed = 150;

    int min_delay = 20;     // 빠를 때 (거의 순간 전환)
    int max_delay = 600;    // 느릴 때 (눈에 확 띄게 느림)

    // 전역 또는 playSequence 함수 내 static 변수
    bool shock_displaying = false;
    std::chrono::steady_clock::time_point shock_trigger_time;

    cv::namedWindow("Speed Simulation", cv::WINDOW_NORMAL);
    cv::setWindowProperty("Speed Simulation", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);

    while (!ctrl_c_pressed.load()) {
        int curr_speed = speed.load();

        // 정규화 (0.0 ~ 1.0)
        float t = float(curr_speed - min_speed) / (max_speed - min_speed);
        t = clamp(t, 0.0f, 1.0f);
        // 체감 효과 극대화용 보간
        t = std::pow(1.0f - t, 5.0f);  // 기존 2.0f → 5.0f로 증폭
        // delay 계산
        int delay_ms = min_delay + t * (max_delay - min_delay);

        cv::Mat frame = frames[frameIndex].clone();

        // 1. 속도 텍스트
        std::string speedText = "Speed : " + std::to_string(curr_speed) + " km/h";
        cv::Point speedPos(30, 40);
        cv::putText(frame, speedText, speedPos, cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 0), 4);
        cv::putText(frame, speedText, speedPos, cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2);

        // 2. Detect 텍스트
        std::string detectLabel = "Detect : ";
        int baseline = 0;
        int fontFace = cv::FONT_HERSHEY_SIMPLEX;
        double fontScale = 0.8;
        int thickness = 2;

        // 위치 계산: Speed 텍스트 바로 아래
        cv::Size speedTextSize = cv::getTextSize(speedText, fontFace, fontScale, thickness, &baseline);
        cv::Point detectLabelPos(speedPos.x, speedPos.y + speedTextSize.height + 10);  // 아래로 10px 띄움

        cv::putText(frame, detectLabel, detectLabelPos, fontFace, fontScale, cv::Scalar(0, 0, 0), 4);
        cv::putText(frame, detectLabel, detectLabelPos, fontFace, fontScale, cv::Scalar(255, 255, 255), 2);

        // 3. Detect 값
        std::string warningText;
        int direction = detected_direction.load();
        if (direction == 1) warningText = "FRONT";
        else if (direction == 2) warningText = "RIGHT";
        else if (direction == 3) warningText = "REAR";
        else if (direction == 4) warningText = "LEFT";
        else warningText = "NONE";

        // Detect 값의 위치: Detect 라벨 바로 오른쪽
        cv::Size detectLabelSize = cv::getTextSize(detectLabel, fontFace, fontScale, thickness, &baseline);
        cv::Point detectValuePos(detectLabelPos.x + detectLabelSize.width + 5, detectLabelPos.y);  // 오른쪽에 5px 간격

        // 테두리 먼저 그림
        cv::putText(frame, warningText, detectValuePos, fontFace, fontScale, cv::Scalar(0, 0, 0), thickness + 2);

        // 본문 색상 조건
        cv::Scalar detectColor = (warningText == "NONE") ? cv::Scalar(255, 255, 255) : cv::Scalar(0, 0, 255);
        cv::putText(frame, warningText, detectValuePos, fontFace, fontScale, detectColor, thickness);

        // loop 내부
        bool shock_flag = shock.load();
        auto now = std::chrono::steady_clock::now();

        // shock 감지되면 타이머 시작
        if (shock_flag && !shock_displaying) {
            shock_displaying = true;
            shock_trigger_time = now;
        }

        // shock 표시 중이면 경과 시간 확인
        if (shock_displaying) {
            auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - shock_trigger_time).count();

            if (elapsed_ms < 2000) {
                // 2초 동안 깜빡이기 (3번: 0~667ms ON, 667~1333ms OFF, 1333~2000ms ON)
                int blink_phase = (elapsed_ms / 333) % 2;  // 0: ON, 1: OFF (333ms ON/OFF)
                if (blink_phase == 0) {
                    // SHOCK 표시
                    std::string shockText = "SHOCK!";
                    int thickness = 3;
                    double fontScale = 1.5;
                    int baseline = 0;
                    int fontFace = cv::FONT_HERSHEY_SIMPLEX;

                    cv::Size textSize = cv::getTextSize(shockText, fontFace, fontScale, thickness, &baseline);
                    cv::Point centerTextPos((frame.cols - textSize.width) / 2,
                                            (frame.rows + textSize.height) / 2);

                    // 테두리 + 본문
                    cv::putText(frame, shockText, centerTextPos, fontFace, fontScale, cv::Scalar(0, 0, 0), thickness + 2);
                    cv::putText(frame, shockText, centerTextPos, fontFace, fontScale, cv::Scalar(0, 0, 255), thickness);
                }
            } else {
                // 2초가 지나면 종료
                shock_displaying = false;
            }
        }

        // 프레임 표시
        cv::imshow("Speed Simulation", frame);
        frameIndex = (frameIndex + 1) % frames.size();

        cv::waitKey(delay_ms);
    }
}

void canReceive(int s) {
    struct can_frame frame;
    memset(&frame, 0, sizeof(struct can_frame));

    struct timeval timeout;
    fd_set read_fds;

    while (!ctrl_c_pressed.load()) {
        FD_ZERO(&read_fds);
        FD_SET(s, &read_fds);

        timeout.tv_sec = 0;
        timeout.tv_usec = 500000;  // 0.5초마다 깨어나서 ctrl_c_pressed 확인

        int ret = select(s + 1, &read_fds, NULL, NULL, &timeout);
        if (ret < 0) {
            perror("select error");
            break;
        } else if (ret == 0) {
            // timeout: 데이터 없음 → 다음 루프로 넘어감
            continue;
        }

        if (FD_ISSET(s, &read_fds)) {
            int nbytes = read(s, &frame, sizeof(frame));
            if (nbytes < 0) {
                perror("read failed");
                continue;
            }

            // CAN 데이터 처리
            if (frame.data[2] == 1) {
                shock.store(true);
            }
            else {
                shock.store(false);
            }

            // 디버깅용 출력 (선택)
            /*
            printf("can_id  = 0x%X\n", frame.can_id);
            printf("can_dlc = %d\n", frame.can_dlc);
            for (int i = 0; i < frame.can_dlc; ++i)
                printf("data[%d] = %d\n", i, frame.data[i]);
            */
        }
    }
}

int main(int argc, const char * argv[]) {

    // LiDAR Variables
	const char * opt_is_channel = NULL; 
	const char * opt_channel = NULL;
    const char * opt_channel_param_first = NULL;
	sl_u32         opt_channel_param_second = 0;
    sl_u32         baudrateArray[2] = {115200, 256000};
    sl_result     op_result;
	int          opt_channel_type = CHANNEL_TYPE_SERIALPORT;
    
	bool useArgcBaudrate = false;

    IChannel* _channel;

    int send_socket, receive_socket;

    if (canSetting(&send_socket, &receive_socket) == -1) {
        return 0;
    }

    printf("Ultra simple LIDAR data grabber for SLAMTEC LIDAR.\n"
           "Version: %s\n", SL_LIDAR_SDK_VERSION);
    
	if (argc>1)
	{ 
		opt_is_channel = argv[1];
	}
	else
	{
		print_usage(argc, argv);
		return -1;
	}

	if(strcmp(opt_is_channel, "--channel")==0){
		opt_channel = argv[2];
		if(strcmp(opt_channel, "-s")==0||strcmp(opt_channel, "--serial")==0)
		{
			// read serial port from the command line...
			opt_channel_param_first = argv[3];// or set to a fixed value: e.g. "com3"
			// read baud rate from the command line if specified...
			if (argc>4) opt_channel_param_second = strtoul(argv[4], NULL, 10);	
			useArgcBaudrate = true;
		}
		else if(strcmp(opt_channel, "-u")==0||strcmp(opt_channel, "--udp")==0)
		{
			// read ip addr from the command line...
			opt_channel_param_first = argv[3];//or set to a fixed value: e.g. "192.168.11.2"
			if (argc>4) opt_channel_param_second = strtoul(argv[4], NULL, 10);//e.g. "8089"
			opt_channel_type = CHANNEL_TYPE_UDP;
		}
		else
		{
			print_usage(argc, argv);
			return -1;
		}
	}
	else
	{
		print_usage(argc, argv);
        return -1;
	}

	if(opt_channel_type == CHANNEL_TYPE_SERIALPORT)
	{
		if (!opt_channel_param_first) {
#ifdef _WIN32
		// use default com port
		opt_channel_param_first = "\\\\.\\com3";
#elif __APPLE__
		opt_channel_param_first = "/dev/tty.SLAB_USBtoUART";
#else
		opt_channel_param_first = "/dev/ttyUSB0";
#endif
		}
	}
    
    // create the driver instance
	ILidarDriver * drv = *createLidarDriver();

    if (!drv) {
        close(send_socket);
        close(receive_socket);
        system("sudo ifconfig can0 down");
        fprintf(stderr, "insufficent memory, exit\n");
        exit(-2);
    }

    sl_lidar_response_device_info_t devinfo;
    bool connectSuccess = false;

    if(opt_channel_type == CHANNEL_TYPE_SERIALPORT){
        if(useArgcBaudrate){
            _channel = (*createSerialPortChannel(opt_channel_param_first, opt_channel_param_second));
            if (SL_IS_OK((drv)->connect(_channel))) {
                op_result = drv->getDeviceInfo(devinfo);

                if (SL_IS_OK(op_result)) 
                {
	                connectSuccess = true;
                }
                else{
                    delete drv;
					drv = NULL;
                }
            }
        }
        else{
            size_t baudRateArraySize = (sizeof(baudrateArray))/ (sizeof(baudrateArray[0]));
			for(size_t i = 0; i < baudRateArraySize; ++i)
			{
				_channel = (*createSerialPortChannel(opt_channel_param_first, baudrateArray[i]));
                if (SL_IS_OK((drv)->connect(_channel))) {
                    op_result = drv->getDeviceInfo(devinfo);

                    if (SL_IS_OK(op_result)) 
                    {
	                    connectSuccess = true;
                        break;
                    }
                    else{
                        delete drv;
					    drv = NULL;
                    }
                }
			}
        }
    }
    else if(opt_channel_type == CHANNEL_TYPE_UDP){
        _channel = *createUdpChannel(opt_channel_param_first, opt_channel_param_second);
        if (SL_IS_OK((drv)->connect(_channel))) {
            op_result = drv->getDeviceInfo(devinfo);

            if (SL_IS_OK(op_result)) 
            {
	            connectSuccess = true;

            }
            else{
                delete drv;
				drv = NULL;
            }
        }
    }

    if (!connectSuccess) {
        (opt_channel_type == CHANNEL_TYPE_SERIALPORT)?
			(fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n"
				, opt_channel_param_first)):(fprintf(stderr, "Error, cannot connect to the specified ip addr %s.\n"
				, opt_channel_param_first));

        onFinished(drv, send_socket, receive_socket);
        return 0;
    }

    // print out the device serial number, firmware and hardware version number..
    printf("SLAMTEC LIDAR S/N: ");
    for (int pos = 0; pos < 16 ;++pos) {
        printf("%02X", devinfo.serialnum[pos]);
    }

    printf("\n"
            "Firmware Ver: %d.%02d\n"
            "Hardware Rev: %d\n"
            , devinfo.firmware_version>>8
            , devinfo.firmware_version & 0xFF
            , (int)devinfo.hardware_version);

    // check health...
    if (!checkSLAMTECLIDARHealth(drv)) {
        onFinished(drv, send_socket, receive_socket);
        return 0;
    }

    signal(SIGINT, ctrlc);
    
	if(opt_channel_type == CHANNEL_TYPE_SERIALPORT)
        drv->setMotorSpeed();

    std::vector<cv::Mat> frames = loadSequence("images", 20);
    if (frames.empty()) {
        printf("Frame load Error.\n");
        return 0;
    }

    std::thread lidarThread(scanLidar, drv, op_result, send_socket);
    std::thread canReceiveThread(canReceive, receive_socket);
    playSequence(frames);

    lidarThread.join();
    canReceiveThread.join();

    cv::destroyAllWindows();
    drv->stop();
	delay(200);
	if(opt_channel_type == CHANNEL_TYPE_SERIALPORT)
        drv->setMotorSpeed(0);
    close(send_socket);
    close(receive_socket);
    system("sudo ifconfig can0 down");

    return 0;
}