// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp>
#include <iostream>
#include <iomanip>
#include <thread>
#include <mutex>
#include <stdio.h>
#include <memory>
#include <functional>
#include <thread>
#include <string.h>
#include <chrono>
#include <signal.h>

bool running = true;

void intHandler(int sig)
{
    signal(SIGINT, SIG_DFL);
    running = false;
}

int main(int argc, char * argv[]) try
{
    std::string outfile = (argc > 1) ? std::string(argv[1]) : std::string("record.bag");

    rs2::pipeline pipe;
    rs2::pipeline_profile selection = pipe.start();
    rs2::device selected_device = selection.get_device();
    std::string camera_name(selected_device.get_info(RS2_CAMERA_INFO_NAME));
    const std::string d435i_name("D435I");
    const std::string t265_name("T265");
    if (camera_name.find(d435i_name) != std::string::npos) {
        std::cout << "Found D435i\n";
    }
    pipe.stop();
/* TODO how to change this option?
    auto depth_sensor = selected_device.first<rs2::depth_sensor>();
    depth_sensor.set_option(RS2_OPTION_AUTO_EXPOSURE_PRIORITY, 0);
*/
    rs2::config cfg;
    cfg.enable_record_to_file(outfile);
    int id = 0;
    cfg.enable_stream(RS2_STREAM_COLOR, 848, 480, RS2_FORMAT_RGB8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 0, 848, 480, RS2_FORMAT_Z16, 30);
    cfg.enable_stream(RS2_STREAM_INFRARED, 1, 848, 480, RS2_FORMAT_Y8, 30);
    cfg.enable_stream(RS2_STREAM_INFRARED, 2, 848, 480, RS2_FORMAT_Y8, 30);
    cfg.enable_stream(RS2_STREAM_ACCEL, 1, 1, RS2_FORMAT_MOTION_XYZ32F, 63);
    cfg.enable_stream(RS2_STREAM_GYRO, 1, 1, RS2_FORMAT_MOTION_XYZ32F, 200);

    std::mutex m;
    auto callback = [&](const rs2::frame& frame)
    {
        std::lock_guard<std::mutex> lock(m);
        auto t = std::chrono::system_clock::now();
        static auto tk = t;
        static auto t0 = t;
        if (t - tk >= std::chrono::seconds(1)) {
            std::cout << "\r" << std::setprecision(3) << std::fixed
                      << "Recording t = "  << std::chrono::duration_cast<std::chrono::seconds>(t-t0).count() << "s" << std::flush;
            tk = t;
        }
    };

    signal(SIGINT, intHandler);
    std::cout << "======================= Press Ctrl+C to stop =======================\n";

    rs2::pipeline_profile profiles = pipe.start(cfg, callback);

    while(running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    std::cout << "\nFinished" << std::endl;

    pipe.stop();

    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
