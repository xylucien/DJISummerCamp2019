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
        dataStack.push(data);
    }

    T pop(){
        T data = dataStack.front();
        dataStack.pop();
        return data;
    }

    void clear(){}

    bool isDataAvailable() const{
        return !dataStack.empty();
    }


private:
    std::queue<T> dataStack;


};


#endif //MANIFOLDCAN_CANTOPIC_H
