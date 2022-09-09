#ifndef __NODELETCLASS_
#define __NODELETCLASS_

#include<nodelet/nodelet.h>

class nodeletclass1:public nodelet::Nodelet//继承父类
{
    public:
        nodeletclass1();//构造函数
    public:
        virtual void onInit();//虚函数，在启动Nodelet节点时，自动调用。
};

#endif