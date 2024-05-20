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

// Function to calculate the steering angle 
double steeringAlgorithm(int direction, double steering, double angularVeloZ, double angularVeloZDerivative)
{
    double steeringAngle = steering; // Initial steering angle

    steeringAngle = (angularVeloZ * 0.002879) + (angularVeloZDerivative * 0.00097);
    // Calculate the steering angle based on angular velocity and its derivative


    if (steeringAngle >= 0)
    {
        steeringAngle = steeringAngle + 0.04;
        if (steeringAngle >= 0.23)
        {
            steeringAngle = 0.23;
        }
        else if (steeringAngle >= 0.19)
        {
            steeringAngle = 0.22;
        }
        else if (steeringAngle >= 0.18)
        {
            steeringAngle = 0.19;
        }
        else if (steeringAngle >= 0.16)
        {
            steeringAngle = 0.17;
        }
        else if (steeringAngle >= 0.12)
        {
            steeringAngle = 0.086;
        }
        else if (steeringAngle >= 0.07)
        {
            steeringAngle = 0.07;
        }
        else if (steeringAngle >= 0.05)
        {
            steeringAngle = 0.06;
        }
        else if (steeringAngle >= 0.02)
        {
            steeringAngle = 0.03;
        }
    }
    else if (direction == 1) // Adjusting steering angle based on direction
    {
        steeringAngle = 0;
    }
    else // Adjusting steering angle for negative values
    {
        steeringAngle = (angularVeloZ * 0.001879) + (angularVeloZDerivative * 0.00091); 
        steeringAngle = steeringAngle - 0.04;
        if (steeringAngle <= -0.23)
        {
            steeringAngle = -0.26;
        }
        else if (steeringAngle <= -0.19)
        {
            steeringAngle = -0.23;
        }
        else if (steeringAngle <= -0.18)
        {
            steeringAngle = -0.222;
        }
        else if (steeringAngle <= -0.16)
        {
            steeringAngle = -0.209;
        }
        else if (steeringAngle <= -0.12)
        {
            steeringAngle = -0.17;
        }
        else if (steeringAngle <= -0.07)
        {
            steeringAngle = -0.11;
        }
    }
    return steeringAngle;
}

auto checkSteering(bool leftCone, bool rightCone, double steeringAngle, double angularVeloZ, double angularVeloZDerivative)
{ // Function to call the steeringAlgorithm with the correct input
    double steering = 0;
    if (leftCone && rightCone) // Both cones detected
    {
        steering = steeringAlgorithm(1, steeringAngle, angularVeloZ, angularVeloZDerivative);
    }
    else if (!leftCone && rightCone) // Only right cone detected
    {
        steering = steeringAlgorithm(0, steeringAngle, angularVeloZ, angularVeloZDerivative);
    }
    else if (!rightCone && leftCone) // Only left cone detected
    {
        steering = steeringAlgorithm(2, steeringAngle, angularVeloZ, angularVeloZDerivative);
    }
    return steering;
}

int32_t main(int32_t argc, char **argv)
{

    int32_t retCode{1};

    double angularVeloZ = 0.0;
    double steeringAngle = 0;
    double angularVeloZDerivative = 0.0;
    
    /*
    int totalComparisons = 0;
    int successfulComparisons = 0;
    double tolerance = 0.25;
    */

    // Parse the command line parameters as we require the user to specify some mandatory information on startup.
    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
    if ((0 == commandlineArguments.count("cid")) ||
        (0 == commandlineArguments.count("name")) ||
        (0 == commandlineArguments.count("width")) ||
        (0 == commandlineArguments.count("height")))
    {
        std::cerr << argv[0] << " attaches to a shared memory area containing an ARGB image." << std::endl;
        std::cerr << "Usage:   " << argv[0] << " --cid=<OD4 session> --name=<name of shared memory area> [--verbose]" << std::endl;
        std::cerr << "         --cid:    CID of the OD4Session to send and receive messages" << std::endl;
        std::cerr << "         --name:   name of the shared memory area to attach" << std::endl;
        std::cerr << "         --width:  width of the frame" << std::endl;
        std::cerr << "         --height: height of the frame" << std::endl;
        std::cerr << "Example: " << argv[0] << " --cid=253 --name=img --width=640 --height=480 --verbose" << std::endl;
    }
    else
    {
        // Extract the values from the command line parameters
        const std::string NAME{commandlineArguments["name"]};
        const uint32_t WIDTH{static_cast<uint32_t>(std::stoi(commandlineArguments["width"]))};
        const uint32_t HEIGHT{static_cast<uint32_t>(std::stoi(commandlineArguments["height"]))};
        const bool VERBOSE{commandlineArguments.count("verbose") != 0};

        // Attach to the shared memory.
        std::unique_ptr<cluon::SharedMemory> sharedMemory{new cluon::SharedMemory{NAME}};
        if (sharedMemory && sharedMemory->valid())
        {
            std::clog << argv[0] << ": Attached to shared memory '" << sharedMemory->name() << " (" << sharedMemory->size() << " bytes)." << std::endl;

            // Interface to a running OpenDaVINCI session where network messages are exchanged.
            // The instance od4 allows you to send and receive messages.
            cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};

            opendlv::proxy::GroundSteeringRequest gsr;
            std::mutex gsrMutex;
            auto onGroundSteeringRequest = [&gsr, &gsrMutex](cluon::data::Envelope &&env)
            {
                // The envelope data structure provide further details, such as sampleTimePoint as shown in this test case:
                // https://github.com/chrberger/libcluon/blob/master/libcluon/testsuites/TestEnvelopeConverter.cpp#L31-L40
                std::lock_guard<std::mutex> lck(gsrMutex);
                gsr = cluon::extractMessage<opendlv::proxy::GroundSteeringRequest>(std::move(env));
                // std::cout << "lambda: groundSteering = " << gsr.groundSteering() << std::endl;
            };

            od4.dataTrigger(opendlv::proxy::GroundSteeringRequest::ID(), onGroundSteeringRequest);

            // Angular velocity reading handler
            opendlv::proxy::AngularVelocityReading angularVZ;
            std::mutex angularVZMutex;
            auto onAngularvelocityReading = [&angularVZ, &angularVZMutex, &angularVeloZ, &angularVeloZDerivative](cluon::data::Envelope &&env)
            {
                std::lock_guard<std::mutex> lck(angularVZMutex);
                angularVZ = cluon::extractMessage<opendlv::proxy::AngularVelocityReading>(std::move(env));
                if (env.senderStamp() == 0)
                {
                    angularVeloZDerivative = angularVZ.angularVelocityZ() - angularVeloZ; // Calculate derivative
                    angularVeloZ = angularVZ.angularVelocityZ();                          // Update angular velocity
                }
                // std::cout << "AVZ = " << angularVZ.angularVelocityZ() << "," << std::endl;
            };

            od4.dataTrigger(opendlv::proxy::AngularVelocityReading::ID(), onAngularvelocityReading);

            // Endless loop; end the program by pressing Ctrl-C.
            while (od4.isRunning())
            {
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

                std::pair<bool, cluon::data::TimeStamp> pair = sharedMemory->getTimeStamp();
                cluon::data::TimeStamp sampleT = pair.second;
                int64_t sampleTimeStamp = cluon::time::toMicroseconds(sampleT);
                // TODO: Here, you can add some code to check the sampleTimePoint when the current frame was captured.
                sharedMemory->unlock();

                cv::rectangle(img, cv::Point(0, 0), cv::Point(650, 250), cv::Scalar(0, 0, 0), cv::FILLED);
                // Placing a black box in the region above the cones to avoid Detecting colours in the background
                cv::rectangle(img, cv::Point(0, 375), cv::Point(650, 500), cv::Scalar(0, 0, 0), cv::FILLED);
                // Placing a black box over the wiring of the car, in order to avoid Detecting colours there

                // Left side black box, extending vertically to cover from the top to the bottom of the existing black boxes
                cv::rectangle(img, cv::Point(0, 0), cv::Point(100, 500), cv::Scalar(0, 0, 0), cv::FILLED);

                // Right side black box, similar to the left, ensuring it covers the same vertical height
                cv::rectangle(img, cv::Point(550, 0), cv::Point(650, 500), cv::Scalar(0, 0, 0), cv::FILLED);

                cv::Mat img_hsv;
                cv::cvtColor(img, img_hsv, cv::COLOR_BGR2HSV);

                // update masking values using further data derived through experimentation with colour-space images
                cv::Scalar blue_lower_boundary = cv::Scalar(78, 50, 50);
                cv::Scalar blue_upper_boundary = cv::Scalar(134, 255, 255);
                // HSV values for the yellow cones
                cv::Scalar yellow_lower_boundary = cv::Scalar(9, 0, 147);
                cv::Scalar yellow_upper_boundary = cv::Scalar(76, 255, 255);

                cv::Mat blue_masking;
                cv::Mat yellow_masking;

                // remove noise and merge individual smaller boxes together within bigger cone box
                cv::Mat mergeBox = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
                cv::Mat closeBox = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(9, 9));

                cv::inRange(img_hsv, yellow_lower_boundary, yellow_upper_boundary, yellow_masking);
                cv::inRange(img_hsv, blue_lower_boundary, blue_upper_boundary, blue_masking);

                // Used for removing smaller noises and merging larger detected objects 
                cv::morphologyEx(blue_masking, blue_masking, cv::MORPH_OPEN, mergeBox);
                cv::morphologyEx(yellow_masking, yellow_masking, cv::MORPH_OPEN, mergeBox);
                cv::morphologyEx(blue_masking, blue_masking, cv::MORPH_CLOSE, closeBox);
                cv::morphologyEx(yellow_masking, yellow_masking, cv::MORPH_CLOSE, closeBox);

                std::vector<std::vector<cv::Point>> blue_contours;
                std::vector<std::vector<cv::Point>> yellow_contours;

                //Finding countours of blue and yellow
                cv::findContours(yellow_masking.clone(), yellow_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
                cv::findContours(blue_masking.clone(), blue_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

                // Dividing the ROI into two halves
                cv::Rect leftRegion(0, 0, 325, 500);
                cv::Rect rightRegion(325, 0, 325, 500);

                int leftSideValue = -1;
                int rightSideValue = -1;

                // Detected cone values
                bool blueDetectedLeft = false;
                bool blueDetectedRight = false;
                bool yellowDetectedLeft = false;
                bool yellowDetectedRight = false;
                bool leftCone = true;
                bool rightCone = true;
                

                for (const auto &blueContour : blue_contours)
                {
                    // Calculate the bounding rectangle of the contour
                    cv::Rect temp_blue_boundary = cv::boundingRect(blueContour);
                    cv::rectangle(img, temp_blue_boundary, cv::Scalar(0, 255, 0), 2);

                    cv::Moments blueMoments = cv::moments(blueContour);
                    cv::Point blueCentroid(static_cast<int>(blueMoments.m10 / blueMoments.m00), static_cast<int>(blueMoments.m01 / blueMoments.m00));
                    // Check if the centroid is in the left region and cones are not detected on the right side
                    if (leftRegion.contains(blueCentroid) && !blueDetectedRight)
                    {
                        leftSideValue = 0; // Blue cones on the left side
                        blueDetectedLeft = true;
                    }
                    else if (rightRegion.contains(blueCentroid) && !blueDetectedLeft)
                    {
                        rightSideValue = 0; // Blue cones on the right side
                        blueDetectedRight = true;
                    }
                }

                for (const auto &yellowContour : yellow_contours)
                {
                    // Calculate the bounding rectangle of the contour
                    cv::Rect temp_yellow_boundary = cv::boundingRect(yellowContour);
                    cv::rectangle(img, temp_yellow_boundary, cv::Scalar(0, 200, 0), 2);

                    cv::Moments yellowMoments = cv::moments(yellowContour); 
                    cv::Point yellowCentroid(static_cast<int>(yellowMoments.m10 / yellowMoments.m00), static_cast<int>(yellowMoments.m01 / yellowMoments.m00));

                    // Check if the centroid is in the left region and cones are not detected on the right side
                    if (leftRegion.contains(yellowCentroid) && !yellowDetectedRight)
                    {
                        leftSideValue = 1; // Yellow cones on the left side
                        yellowDetectedLeft = true;
                    }
                    else if (rightRegion.contains(yellowCentroid) && !yellowDetectedLeft)
                    {
                        rightSideValue = 1; // Yellow cones on the right side
                        yellowDetectedRight = true;
                    }
                }

                if (!blueDetectedLeft && !yellowDetectedLeft)
                {
                    leftCone = false; // Make the left cone false if no cones are detected on that side
                }

                if (!blueDetectedRight && !yellowDetectedRight)
                {
                    rightCone = false; // Make the right cone false if no cones are detected on that side
                }

                steeringAngle = checkSteering(leftCone, rightCone, steeringAngle, angularVeloZ, angularVeloZDerivative);
                // Calling the checkSteering function to call the steering angle with the right input

                // TODO: Do something with the frame.
                // Example: Draw a red rectangle and display image.
                // cv::rectangle(img, cv::Point(50, 50), cv::Point(100, 100), cv::Scalar(0,0,255));

                // If you want to access the latest received ground steering, don't forget to lock the mutex:
                {
                    /*
                    std::lock_guard<std::mutex> lck(gsrMutex);
                    if (groundSteering != 0)
                    {
                        totalComparisons++;
                        double lowerBound = groundSteering * (1 - tolerance);
                        double upperBound = groundSteering * (1 + tolerance);
                        if (steeringAngle >= lowerBound && steeringAngle <= upperBound)
                        {
                            successfulComparisons++;
                        }
                    }    
                    */
                    //std::cout << "main: groundSteering: " << gsr.groundSteering() << std::endl;
                    //std::cout << "our: " << steeringAngle << std::endl;
                    std::cout << "Group_15;" << sampleTimeStamp << ";" << steeringAngle << std::endl;
                }

                // Display image on your screen.
                if (VERBOSE)
                {
                    cv::imshow(sharedMemory->name().c_str(), img);
                    cv::waitKey(1);
                }
            }
            
            /*
            if (totalComparisons > 0)
            {
                double accuracy = (static_cast<double>(successfulComparisons) / totalComparisons) * 100.0;
                std::cout << "Accuracy of steering algorithm: " << accuracy << "%" << std::endl;
            }
            else
            {
                std::cout << "No valid comparisons made." << std::endl;
            }
            
            */
        }
        retCode = 0;
    }
    return retCode;
}