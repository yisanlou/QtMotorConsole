#include "QtMotorConsole.h"
#include <QtWidgets/QApplication>

// 系统头文件
#include <iostream>
#include <string>
#include <vector>
#include <Windows.h>
#include "MultiCardCPP.h"
#include <atomic>
#include <thread>
#include <map>

#pragma comment(lib,"D:\\QtMotorConsole\\MultiCard.lib")

#pragma execution_character_set("utf-8")

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    QtMotorConsole window;
    window.show();
    return app.exec();
}
