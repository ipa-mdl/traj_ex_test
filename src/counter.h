#ifndef TRAJ_EX_TEST_DEBUG_COUNTER_H
#define TRAJ_EX_TEST_DEBUG_COUNTER_H

#include <ros/ros.h>

class RateCounter {
public:
    RateCounter() { reset(); }
    size_t count(ros::Time now, size_t report_every_n=100){
        ++counter_;
        if(counter_ > 1){
            rate_ = counter_/(now-start_).toSec();
            if(report_every_n && (counter_ % report_every_n) == 0){
                print();
            }
        }else{
            start_ = now;
        }
        return counter_;
    }
    void reset(){
        counter_ = 0;
        rate_ = 0;
    }
    void print(){
        ROS_INFO_STREAM("Current rate " << rate_ << " Hz, based on " << counter_ << " counts");
    }

    friend std::ostream& operator<< (std::ostream& stream, const RateCounter& rc) {
        return stream << "Current rate " << rc.rate_ << " Hz, based on " << rc.counter_ << " counts";
    }
private:
    ros::Time start_;
    double rate_;
    size_t counter_;
};

class LatencyCounter {
public:
    LatencyCounter() { reset(); }

    size_t count(ros::Time now, ros::Time stamp, size_t report_every_n=100){
        size_t counter = rate_counter_.count(now, 0);

        latency_last_ = (now-stamp).toSec();
        if(latency_last_ > latency_max_) latency_max_ = latency_last_;

        latency_total_ += latency_last_;
        latency_average_ = latency_total_/counter;

        if(report_every_n && (counter % report_every_n) == 0){
            print();
        }
        return counter;
    }
    void reset(){
        rate_counter_.reset();
        latency_last_ = latency_max_ = latency_total_ = latency_average_ = 0.0;
    }
    void print() {
        rate_counter_.print();
        ROS_INFO_STREAM("Latency (s): " << latency_last_ << " current, " << latency_average_ << " average, " << latency_max_ << " maximum");
    }
    friend std::ostream& operator<< (std::ostream& stream, const LatencyCounter& lc) {
        return stream << "Latency (s): " << lc.latency_last_ << " current, " << lc.latency_average_ << " average, " << lc.latency_max_ << " maximum";

    }

private:
    RateCounter rate_counter_;
    double latency_last_, latency_max_, latency_total_, latency_average_;
};

#endif // !TRAJ_EX_TEST_DEBUG_COUNTER_H