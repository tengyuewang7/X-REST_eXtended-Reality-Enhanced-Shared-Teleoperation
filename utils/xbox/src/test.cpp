#include <iostream>
#include <vector>
#include <thread>
#include <mutex>
#include <memory>

std::shared_ptr<std::vector<int>> myVector = std::make_shared<std::vector<int>>();
std::mutex myMutex;

void add_to_vector(int x) {
    while (true) {
        { 
            std::lock_guard<std::mutex> lock(myMutex); 
            for (int i = 0; i < 1000; ++i) {
                myVector->push_back(x);
            }
            for (int i = 0; i < 1000 - 1; ++i) {
                myVector->pop_back(); 
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10)); 
    }
}

void size_of_vector() {
    while (true) {
        // std::lock_guard<std::mutex> lock(myMutex);
        // std::cout << "Size of vector: " << myVector.size() << std::endl;
    }
}

std::vector<int> get_vector() {
    return *myVector; 
}

int main() {
    std::thread thread1(add_to_vector, 5);
    std::thread thread2(add_to_vector, 10);

    auto now = std::chrono::system_clock::now();

    // 转换为毫秒
    auto start_time = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();

    // std::thread thread3(size_of_vector);
    while (true) {

        auto now = std::chrono::system_clock::now();

        // 转换为毫秒
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();

        // 输出当前时间
        std::cout << "Current time (ms): " << (ms - start_time) / 1000.0 << std::endl;

        { 
            std::lock_guard<std::mutex> lock(myMutex); 
            std::cout << "Size of vector: " << myVector->size() << std::endl;
        } 
        std::this_thread::sleep_for(std::chrono::seconds(1)); 
    }

    thread1.join();
    thread2.join();
    // thread3.join();

    return 0;
}
