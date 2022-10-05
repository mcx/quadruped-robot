// The MIT License

// Copyright (c) 2022
// Robot Motion and Vision Laboratory at East China Normal University
// Contact: tophill.robotics@gmail.com

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef QR_TIMER_H
#define QR_TIMER_H
#include <time.h>
#include <assert.h>
#include <stdint.h>
#include <ros/ros.h>

/**
 * @brief The Timer class counts the time when robots started
 */
class Timer {
public:
    /**
     * @brief constructor of Timer
     */
    Timer(){
        start = clock();
        startTime = (double)start;
    }

    virtual ~Timer() = default;
    /**
     * @brief get time since robot reset
     * @return time since reset
     */
    virtual double GetTimeSinceReset(){
        finish = clock();
        double timeSinceReset = (double)(finish - startTime) / CLOCKS_PER_SEC; // second(s)
        return timeSinceReset;
    }

    /**
     * @brief set current time as start time
     */
    virtual double ResetStartTime(){
        start = clock();
        startTime = (double)start;
        return startTime;
    }

private:

    /**
     * @brief start time
     */
    clock_t start;

    /**
     * @brief finish time
     */
    clock_t finish;

    /**
     * @brief start time
     */
    double startTime;
};

class RosTimer : public Timer {
    public:
        RosTimer() {
            startRos = ros::Time::now().toSec();
        }
        virtual ~RosTimer() = default;
        virtual double GetTimeSinceReset() {
            double timeSinceReset = ros::Time::now().toSec() - startRos;
            return timeSinceReset;
        }
        virtual double ResetStartTime() {
            startRos = ros::Time::now().toSec();
            return startRos;
        }
    private:
        double startRos;
};

class TimerInterface {
    public:
        TimerInterface(bool useRosTimerIn = false) : useRosTime(useRosTimerIn), timePtr(nullptr) {
            if (useRosTime) {
                timePtr = new RosTimer();
            }else {
                timePtr = new Timer();
            }

            startTime = 0;
            timeSinceReset = 0;
        }

        ~TimerInterface() {
            delete timePtr;
        }
        double GetTimeSinceReset() {
            timeSinceReset = timePtr->GetTimeSinceReset();
            return timeSinceReset;
        }
        void ResetStartTime() {
            startTime = timePtr->ResetStartTime();
        }
    private:
        bool useRosTime;
        Timer* timePtr;
        double startTime;
        double timeSinceReset;
};

#endif // QR_TIMER_H
