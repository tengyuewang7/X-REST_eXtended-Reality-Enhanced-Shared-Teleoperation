#ifndef _XBOX_MONITOR_HPP_ 
#define _XBOX_MONITOR_HPP_

#include <stdio.h>  
#include <unistd.h>  
#include <string.h>  
#include <sys/types.h>  
#include <sys/stat.h>  
#include <fcntl.h>  
#include <errno.h>  
#include <linux/input.h>  
#include <linux/joystick.h>  
#include <iostream>
#include <thread> 
#include <functional>
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>

/**
 * @brief Data struct used to store input of xbox. 
*/
struct xbox_map
{
    int     time;
    int     a;
    int     b;
    int     x;
    int     y;
    int     lb;
    int     rb;
    int     start;
    int     back;
    int     home;
    int     lo;
    int     ro;
 
    int     lx;
    int     ly;
    int     rx;
    int     ry;
    int     lt;
    int     rt;
    int     xx;
    int     yy;
 
}; 

/**
 * @brief Operator overloading for printing struct xbox_map. 
*/
std::ostream& operator<<(std::ostream& os, 
                         const xbox_map& map) {
    os << "Time: " << map.time
       << ", A: " << map.a
       << ", B: " << map.b
       << ", X: " << map.x
       << ", Y: " << map.y
       << ", LB: " << map.lb
       << ", RB: " << map.rb
       << ", start: " << map.start
       << ", back: " << map.back
       << ", home: " << map.home
       << ", LO: " << map.lo
       << ", RO: " << map.ro
       << ", XX: " << map.xx
       << ", YY: " << map.yy
       << ", LX: " << map.lx
       << ", LY: " << map.ly
       << ", RX: " << map.rx
       << ", RY: " << map.ry
       << ", LT: " << map.lt
       << ", RT: " << map.rt;

    return os;
}


/**
 * @brief Class to read and store xbox input. 
*/
class XboxMonitor {
public: 

/**
 * @brief Constructor. 
*/
XboxMonitor() { 
    // Open the xbox port 
    xbox_fd_ = open("/dev/input/js0", O_RDONLY);
    if (xbox_fd_ < 0) {
        throw std::system_error(errno, std::system_category(), 
            "Failed to open /dev/input/js0");
    }

    // Initialize shared_ptr for storing xbox input 
    xbox_input_ = std::make_shared<xbox_map>(); 
    xbox_input_->lt = XBOX_AXIS_VAL_MIN; 
    xbox_input_->rt = XBOX_AXIS_VAL_MIN; 
} 

/**
 * @brief Deconstructor. 
*/
~XboxMonitor() { 
    // Stop and close the xbox port
    stop(); 
    if (xbox_fd_ >= 0) { close(xbox_fd_); }
}

/**
 * @brief Start a thread to keep monitoring xbox input. 
*/
void start() { 
    running_ = true; 
    running_thread_ = std::thread(&XboxMonitor::update, this);
}

/**
 * @brief Close the xbox input monitoring thread. 
*/
void stop() { 
    if (running_) {
        running_ = false; 
        if (running_thread_.joinable()) {
            running_thread_.join(); 
        }
    }
}

/**
 * @brief Return the value of current xbox input. 
*/
xbox_map get_map() {
    std::lock_guard<std::mutex> lock_xbox_input(xbox_input_mutex_); 
    return *xbox_input_; 
}

private: 
/**
 * @brief Keep monitoring xbox input. 
*/
void update() { 
    while(running_)
    {
        xbox_input_read();
    }
}

/**
 * @brief Read the xbox input and store. 
*/
void xbox_input_read() {

    int len, type, number, value;
    struct js_event js;
    
    // Get data from xbox port and store in js 
    len = read(xbox_fd_, &js, sizeof(struct js_event)); 
    // Check if the data is valid 
    if (len < 0)
    {
        std::cout << "Read errors" << std::endl;
    }

    type = js.type;
    number = js.number;
    value = js.value; 

    // Lock xbox_input_ when assigning read-in value 
    std::lock_guard<std::mutex> lock_xbox_input(xbox_input_mutex_); 
    
    // Store data from js 
    xbox_input_->time = js.time; 

    if (type == JS_EVENT_BUTTON)
    {
        switch (number)
        {
            case XBOX_BUTTON_A:
                xbox_input_->a = value;
                break;
 
            case XBOX_BUTTON_B:
                xbox_input_->b = value;
                break;
 
            case XBOX_BUTTON_X:
                xbox_input_->x = value;
                break;
 
            case XBOX_BUTTON_Y:
                xbox_input_->y = value;
                break;
 
            case XBOX_BUTTON_LB:
                xbox_input_->lb = value;
                break;
 
            case XBOX_BUTTON_RB:
                xbox_input_->rb = value;
                break;
 
            case XBOX_BUTTON_START:
                xbox_input_->start = value;
                break;
 
            case XBOX_BUTTON_BACK:
                xbox_input_->back = value;
                break;
 
            case XBOX_BUTTON_HOME:
                xbox_input_->home = value;
                break;
 
            case XBOX_BUTTON_LO:
                xbox_input_->lo = value;
                break;
 
            case XBOX_BUTTON_RO:
                xbox_input_->ro = value;
                break;
 
            default:
                break;
        }
    }
    else if (type == JS_EVENT_AXIS)
    {
        switch(number)
        {
            case XBOX_AXIS_LX:
                xbox_input_->lx = value;
                break;
 
            case XBOX_AXIS_LY:
                xbox_input_->ly = value;
                break;
 
            case XBOX_AXIS_RX:
                xbox_input_->rx = value;
                break;
 
            case XBOX_AXIS_RY:
                xbox_input_->ry = value;
                break;
 
            case XBOX_AXIS_LT:
                xbox_input_->lt = value;
                break;
 
            case XBOX_AXIS_RT:
                xbox_input_->rt = value;
                break;
 
            case XBOX_AXIS_XX:
                xbox_input_->xx = value;
                break;
 
            case XBOX_AXIS_YY:
                xbox_input_->yy = value;
                break;
 
            default:
                break;
        }
    }
    else if (type == JS_EVENT_INIT)
    {
        /* Init do nothing */
    }
}

// Const varibales for buttons and axes  
static constexpr int XBOX_TYPE_BUTTON = 0x01; 
static constexpr int XBOX_TYPE_AXIS   = 0x02;  

static constexpr int XBOX_BUTTON_A     = 0x00;  
static constexpr int XBOX_BUTTON_B     = 0x01;  
static constexpr int XBOX_BUTTON_X     = 0x02;  
static constexpr int XBOX_BUTTON_Y     = 0x03;  
static constexpr int XBOX_BUTTON_LB    = 0x04;  
static constexpr int XBOX_BUTTON_RB    = 0x05;  
static constexpr int XBOX_BUTTON_START = 0x06;  
static constexpr int XBOX_BUTTON_BACK  = 0x07;  
static constexpr int XBOX_BUTTON_HOME  = 0x08;  
static constexpr int XBOX_BUTTON_LO    = 0x09;    /* 左摇杆按键 */  
static constexpr int XBOX_BUTTON_RO    = 0x0a;    /* 右摇杆按键 */  

static constexpr int XBOX_BUTTON_ON    = 0x01;  
static constexpr int XBOX_BUTTON_OFF   = 0x00;  

static constexpr int XBOX_AXIS_LX      = 0x00;    /* 左摇杆X轴 */  
static constexpr int XBOX_AXIS_LY      = 0x01;    /* 左摇杆Y轴 */  
static constexpr int XBOX_AXIS_RX      = 0x03;    /* 右摇杆X轴 */  
static constexpr int XBOX_AXIS_RY      = 0x04;    /* 右摇杆Y轴 */  
static constexpr int XBOX_AXIS_LT      = 0x02;  
static constexpr int XBOX_AXIS_RT      = 0x05;  
static constexpr int XBOX_AXIS_XX      = 0x06;    /* 方向键X轴 */  
static constexpr int XBOX_AXIS_YY      = 0x07;    /* 方向键Y轴 */  

static constexpr int XBOX_AXIS_VAL_UP    = -32767;  
static constexpr int XBOX_AXIS_VAL_DOWN  =  32767;  
static constexpr int XBOX_AXIS_VAL_LEFT  = -32767;  
static constexpr int XBOX_AXIS_VAL_RIGHT =  32767;  

static constexpr int XBOX_AXIS_VAL_MIN = -32767;  
static constexpr int XBOX_AXIS_VAL_MAX =  32767;  
static constexpr int XBOX_AXIS_VAL_MID =  0x00;  

// Xbox port id
int xbox_fd_; 
// Shared_ptr used to store xbox input 
std::shared_ptr<xbox_map> xbox_input_; 
// Thread for keeping monitoring and its flag 
std::thread running_thread_;
std::atomic<bool> running_; 
// Mutex to manage variabels shared in different threads 
std::mutex xbox_input_mutex_; 

}; 

#endif

