#include "../Tasks/Inc/task_iwdg.h"

#include "spiflash.h"

extern u8 isRxNewFirmware;

extern osSemaphoreId semCreateWebPckgHandle;

u16 iwdgTaskReg;
u32 iwdgErrCount;

void taskManageIWDG(void const* argument) {
    iwdgTaskReg = 0;
    iwdgErrCount = 0;
    u32 timeout = 0;

    for (;;) {
        if ((isRxNewFirmware && (iwdgTaskReg & IWDG_TASK_REG_NEW_BIN) == IWDG_TASK_REG_NEW_BIN) ||
            (!isRxNewFirmware && (iwdgTaskReg & IWDG_TASK_REG_ALL) == IWDG_TASK_REG_ALL)) {
            iwdgTaskReg = 0;
            iwdgErrCount = 0;
        } else {
            iwdgErrCount++;
        }

        if (iwdgErrCount > 800) {
            spiFlashSaveInfo();
            NVIC_SystemReset();
        }

        if (!(timeout % 30)) {
            osSemaphoreRelease(semCreateWebPckgHandle);
        }

        LL_IWDG_ReloadCounter(IWDG);
        osDelay(3000);
        timeout += 3;
        HAL_GPIO_TogglePin(LED3G_GPIO_Port, LED3G_Pin);
    }
}