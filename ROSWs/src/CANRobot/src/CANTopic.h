//
// Created by Abiel on 7/31/19.
//

#ifndef MANIFOLDCAN_CANTOPIC_H
#define MANIFOLDCAN_CANTOPIC_H

#include <list>
#include <queue>
#include <mutex>

template <typename T>
class CANTopic {
public:
    CANTopic(){
        ;
    }

    void push(const T &data){
        std::lock_guard<std::mutex> lock(dataMutex);
        dataStack.push(data);
    }

    T pop(){
        std::lock_guard<std::mutex> lock(dataMutex);
        T data = dataStack.front();
        dataStack.pop();
        return data;
    }

    bool isDataAvailable() const{
        std::lock_guard<std::mutex> lock(dataMutex);
        return !dataStack.empty();
    }


private:
    std::mutex dataMutex;
    std::queue<T> dataStack;


};


#endif //MANIFOLDCAN_CANTOPIC_H
