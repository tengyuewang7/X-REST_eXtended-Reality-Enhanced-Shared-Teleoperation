#include "XboxMonitor.hpp"
#include "RizonXbox.hpp"

int main(void)  
{  
    RizonXbox rx; 
    rx.start(); 


    auto timer = std::chrono::high_resolution_clock::now();

    while (true) {

        timer += std::chrono::milliseconds(10);

        auto res = rx.get_pose(); 
        auto res_io = rx.get_io();
        auto map = rx.get_map();  

        for (const auto& i : res_io) {
            std::cout << i << " "; 
        }
        std::cout << std::endl; 

        std::cout << map << std::endl; 

        std::this_thread::sleep_until(timer);

    } 
    return 0;  
}  

