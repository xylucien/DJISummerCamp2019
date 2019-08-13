#include <iostream>
#include <ros/ros.h>
#include "serial/serial.h"
#include "rm_protocol.h"
#include <simple_msgs/GameStatus.h>

ros::Publisher gameStatusPub;

void RMUnpackCallback(uint8_t* data) {
    data_frame_unpack_struct *unpack = (data_frame_unpack_struct *)data;
    switch(unpack->cmd_id) {
        case GAME_STATUS_CMD_ID: {
            summer_camp_info_t *info = (summer_camp_info_t *) unpack->cmd_data;

            simple_msgs::GameStatus status;

            //round info
            status.round_team = info->round_team;
            status.round_remain_tick = info->round_remain_tick;
            status.round_remain_cnt = info->round_remain_cnt;

            status.red_score = info->realtime_score[0];
            status.blue_score = info->realtime_score[1];

            status.red_position_x = info->car_location[0] & 0xF;
            status.red_position_y = (info->car_location[0] >> 4) & 0xF;

            status.blue_position_x = info->car_location[1] & 0xF;
            status.blue_position_y = (info->car_location[1] >> 4) & 0xF;

            //Castle
            status.castle_energy.resize(7);
            for(size_t i = 0; i < 7; i++){
                status.castle_energy[i].red_energy = info->castle_energy[i].energy[0];
                status.castle_energy[i].blue_energy = info->castle_energy[i].energy[1];
            }

            //cell info
            status.region_occupy.resize(7 * 9);
            int currentId = 0;

            for(int x = 0; x < 7; x++){
                for(int y = 0; y < 9; y++){
                    status.region_occupy[currentId].status = info->region_occupy[y][x].status;
                    status.region_occupy[currentId].team = info->region_occupy[y][x].belong;

                    currentId++;
                }
            }


            gameStatusPub.publish(status);
            break;
        }
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "RefereeComms");

    ros::NodeHandle nh;
    gameStatusPub = nh.advertise<simple_msgs::GameStatus>("gameStatus", 100);

    serial::Serial serial;
    serial::Timeout timeout = serial::Timeout::simpleTimeout(100);

    serial.setPort("/dev/ttyTHS2");
    serial.setBaudrate(115200);
    serial.setTimeout(timeout);

    while(!serial.isOpen()) {
        try {
            serial.open();
        }
        catch(const std::exception& e) {
            ROS_ERROR("Judge Serial cannot open [%s].", e.what());
        }

        ros::Duration(0.5).sleep();
    }

    //Init unpack obj
    unpack_data_t unpack_obj;
    unpack_obj.index = 0;
    unpack_obj.unpack_step = STEP_HEADER_SOF;

    ros::Rate r(100);

    while(ros::ok()) {
        if(serial.waitReadable()) {
            std::cout << "hi" << std::endl;
            //read to buffer
            std::vector<uint8_t> buffer;
            serial.read(buffer, serial.available());

            //unpack
            for(int idx = 0; idx < buffer.size(); idx++) {
                RMProtocolUnpack(&unpack_obj, buffer[idx], &RMUnpackCallback);
            }
        }

        r.sleep();
    }

  return 0;
}
