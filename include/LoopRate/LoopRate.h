//
// Created by bob on 02-10-19.
//

#ifndef NAVIGATION_LOOPRATE_H
#define NAVIGATION_LOOPRATE_H

#include <iostream>
#include <chrono>
#include <unistd.h>

class LoopRate {
public:
    std::chrono::time_point<std::chrono::high_resolution_clock> prevtime;
    int rate;
    long periodMilliseconds;
    unsigned long long elapsedTime = 0;
    double periodSeconds;
    LoopRate(int rate_){
        rate = rate_;
        periodMilliseconds = long(1000/rate);
        periodSeconds = 1/double(rate);
        prevtime = std::chrono::high_resolution_clock::now();
        std::cout << "Loop rate: " << rate << " Hz" << std::endl;
    }
    void update(){
        auto curtime = std::chrono::high_resolution_clock::now();
        long timediff = std::chrono::duration_cast<std::chrono::milliseconds>(curtime - prevtime).count();
        long timeshortage = periodMilliseconds - timediff;
        //cout << int(100*((double)timediff/(double)periodMilliseconds)) << "% busy" << endl;
        if(timeshortage > 0){
            usleep(1000*timeshortage);
            elapsedTime += periodMilliseconds;
        }else{
            if(elapsedTime != 0) {
                //std::cout << "Rate not achieved" << std::endl;
            }
            elapsedTime += timediff;
        }
        prevtime = std::chrono::high_resolution_clock::now();
    }
};

#endif //NAVIGATION_LOOPRATE_H
