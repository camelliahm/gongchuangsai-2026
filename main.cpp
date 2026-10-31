#include "board.h"
#include "can.h"
#include "contract.h"
#include "servo.h"
#include "input.h"
#include "vision.h"
#include "screen.h"
#include "oled.h"
#include "nav.h"

#include <cassert>
#include <thread>

static void App_TaskCreate(void)
{
    static std::thread tCtrl(contract_CtrlTask);
    static std::thread tCanRx(contract_CanRecvTask);
    static std::thread tState(contract_StateTask);

    /* 任务永不退出：join 只是挂住主线程，等价原调度器不返回 */
    tCtrl.join();
    tCanRx.join();
    tState.join();
}

static void ALL_Init(void)
{
    board_init();                          /* Linux 下无时钟/GPIO 需配置，空实现 */
    servo_Init();
    input_Init();
    vision_Init();
    screen_Init();
    oled_Init();
    nav_Init();
    assert(can_RtosInit() == true);
    contract_Init();
}

int main(void)
{
    ALL_Init();
    App_TaskCreate();

    return 0;
}
