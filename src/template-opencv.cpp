/*
 * Copyright (C) 2020  Christian Berger
 *
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
 */

// Include the single-file, header-only middleware libcluon to create high-performance microservices
#include "cluon-complete.hpp"
// Include the OpenDLV Standard Message Set that contains messages that are usually exchanged for automotive or robotic applications 
#include "opendlv-standard-message-set.hpp"

// Include the GUI and image processing header files from OpenCV
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

int32_t main(int32_t argc, char **argv) {
    int32_t retCode{1};
    double leftInfrared;
    double rightInfrared;
    // Parse the command line parameters as we require the user to specify some mandatory information on startup.
    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
    if ( (0 == commandlineArguments.count("cid")) ||
         (0 == commandlineArguments.count("name")) ||
         (0 == commandlineArguments.count("width")) ||
         (0 == commandlineArguments.count("height")) ) {
        std::cerr << argv[0] << " attaches to a shared memory area containing an ARGB image." << std::endl;
        std::cerr << "Usage:   " << argv[0] << " --cid=<OD4 session> --name=<name of shared memory area> [--verbose]" << std::endl;
        std::cerr << "         --cid:    CID of the OD4Session to send and receive messages" << std::endl;
        std::cerr << "         --name:   name of the shared memory area to attach" << std::endl;
        std::cerr << "         --width:  width of the frame" << std::endl;
        std::cerr << "         --height: height of the frame" << std::endl;
        std::cerr << "Example: " << argv[0] << " --cid=253 --name=img --width=640 --height=480 --verbose" << std::endl;
    }
    else {
        // Extract the values from the command line parameters
        const std::string NAME{commandlineArguments["name"]};
        const uint32_t WIDTH{static_cast<uint32_t>(std::stoi(commandlineArguments["width"]))};
        const uint32_t HEIGHT{static_cast<uint32_t>(std::stoi(commandlineArguments["height"]))};
        const bool VERBOSE{commandlineArguments.count("verbose") != 0};

        // Attach to the shared memory.
        std::unique_ptr<cluon::SharedMemory> sharedMemory{new cluon::SharedMemory{NAME}};
        if (sharedMemory && sharedMemory->valid()) {
            std::clog << argv[0] << ": Attached to shared memory '" << sharedMemory->name() << " (" << sharedMemory->size() << " bytes)." << std::endl;

            // Interface to a running OpenDaVINCI session where network messages are exchanged.
            // The instance od4 allows you to send and receive messages.
            cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};

            opendlv::proxy::GroundSteeringRequest gsr;
            opendlv::proxy::VoltageReading infrared;
            std::mutex infraredM;
            std::mutex gsrMutex;
            auto onGroundSteeringRequest = [&gsr, &gsrMutex](cluon::data::Envelope &&env){
                // The envelope data structure provide further details, such as sampleTimePoint as shown in this test case:
                // https://github.com/chrberger/libcluon/blob/master/libcluon/testsuites/TestEnvelopeConverter.cpp#L31-L40
                std::lock_guard<std::mutex> lck(gsrMutex);
                gsr = cluon::extractMessage<opendlv::proxy::GroundSteeringRequest>(std::move(env));
                std::cout << "lambda: groundSteering = " << gsr.groundSteering() << std::endl;
            };

            od4.dataTrigger(opendlv::proxy::GroundSteeringRequest::ID(), onGroundSteeringRequest);

            //infrared retreiving

            auto voltageReading = [&infrared, &infraredM, &rightInfrared, &leftInfrared](cluon::data::Envelope &&env){
                std::lock_guard<std::mutex> lck(infraredM);
                infrared = cluon::extractMessage<opendlv::proxy::VoltageReading>(std::move(env));
                if(env.senderStamp()==3){
                    rightInfrared = infrared.voltage();
                }
                else if (env.senderStamp()==1)
                {
                    leftInfrared = infrared.voltage();
                }    
            };

            od4.dataTrigger(opendlv::proxy::VoltageReading::ID(), voltageReading);

            // Endless loop; end the program by pressing Ctrl-C.
            while (od4.isRunning()) {
                // OpenCV data structure to hold an image.
                cv::Mat img;

                // Wait for a notification of a new frame.
                sharedMemory->wait();

                // Lock the shared memory.
                sharedMemory->lock();
                {
                    // Copy the pixels from the shared memory into our own data structure.
                    cv::Mat wrapped(HEIGHT, WIDTH, CV_8UC4, sharedMemory->data());
                    img = wrapped.clone();
                }
                // TODO: Here, you can add some code to check the sampleTimePoint when the current frame was captured.
                sharedMemory->unlock();

                cv::rectangle(img, cv::Point(0, 0), cv::Point(650, 250), cv::Scalar(0,0,0), cv::FILLED);
                // Placing a black box in the region above the cones to avoid Detecting colours in the background
                cv::rectangle(img, cv::Point(0, 385), cv::Point(650, 500), cv::Scalar(0,0,0), cv::FILLED);
                // Placing a black box over the wiring of the car, in order to avoid Detecting colours there 

                cv:: Mat img_hsv;
                cv::cvtColor(img,img_hsv,cv::COLOR_BGR2HSV);
               
                // update masking values using further data derived through experimentation with colour-space images
                cv::Scalar blue_lower_boundary = cv::Scalar(78, 50, 50); 
                cv::Scalar blue_upper_boundary = cv::Scalar(134, 255, 255); 
                //HSV values for the yellow cones
                cv::Scalar yellow_lower_boundary = cv::Scalar(9,0,147);
                cv::Scalar yellow_upper_boundary = cv::Scalar(76,255,255);

                cv::Mat blue_masking;
                cv::Mat yellow_masking;

                //remove noise and merge individual smaller boxes together within bigger cone box
                cv::Mat mergeBox = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(5,5));

                cv::inRange(img_hsv,yellow_lower_boundary,yellow_upper_boundary,yellow_masking);
                cv::inRange(img_hsv,blue_lower_boundary,blue_upper_boundary,blue_masking);
                cv::morphologyEx(blue_masking, blue_masking, cv::MORPH_OPEN, mergeBox);
                cv::morphologyEx(yellow_masking, yellow_masking, cv::MORPH_OPEN, mergeBox);
                

                std::vector<std::vector<cv::Point>> blue_contours;
                std::vector<std::vector<cv::Point>> yellow_contours;
                cv::findContours(yellow_masking.clone(),yellow_contours,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE);
                cv::findContours(blue_masking.clone(),blue_contours,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE);

                for(const auto &New_blue_contours : blue_contours){
                    cv::Rect temp_blue_boundary = cv::boundingRect(New_blue_contours);
                    cv::rectangle(img,temp_blue_boundary,cv::Scalar(255,255,0),2);
                }
                for(const auto &New_yellow_contours : yellow_contours){
                    cv::Rect temp_yellow_boundary = cv::boundingRect(New_yellow_contours);
                    cv::rectangle(img,temp_yellow_boundary,cv::Scalar(0,255,255),2);
                }


                // TODO: Do something with the frame.
                // Example: Draw a red rectangle and display image.
                // cv::rectangle(img, cv::Point(50, 50), cv::Point(100, 100), cv::Scalar(0,0,255));

                // If you want to access the latest received ground steering, don't forget to lock the mutex:
                {
                    std::lock_guard<std::mutex> lck(gsrMutex);
                    std::cout << "main: groundSteering = " << gsr.groundSteering() << std::endl;
                }

                // Display image on your screen.
                if (VERBOSE) {
                    cv::imshow(sharedMemory->name().c_str(), img);
                    cv::waitKey(1);
                }
            }
        }
        retCode = 0;
    }
    return retCode;
}