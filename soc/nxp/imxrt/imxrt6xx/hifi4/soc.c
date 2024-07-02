#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <fsl_clock.h>
#include <fsl_common.h>

#define BOARD_XTAL_SYS_CLK_HZ 24000000U

static int mimxrt685s_hifi4_init(void)
{
	CLOCK_SetXtalFreq(BOARD_XTAL_SYS_CLK_HZ);
	CLOCK_EnableClock(kCLOCK_Dmac1);

#if (DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(flexcomm1), nxp_lpc_i2s, okay) && CONFIG_I2S)
	/* attach AUDIO PLL clock to FLEXCOMM1 (I2S_PDM) */
	CLOCK_AttachClk(kAUDIO_PLL_to_FLEXCOMM1);
#endif

#if (DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(flexcomm3), nxp_lpc_i2s, okay) && CONFIG_I2S)
	/* attach AUDIO PLL clock to FLEXCOMM1 (I2S_PDM) */
	CLOCK_AttachClk(kAUDIO_PLL_to_FLEXCOMM3);
#endif

#if CONFIG_AUDIO_CODEC_WM8904
	/* attach AUDIO PLL clock to MCLK */
	CLOCK_AttachClk(kAUDIO_PLL_to_MCLK_CLK);
	CLOCK_SetClkDiv(kCLOCK_DivMclkClk, 1);
	SYSCTL1->MCLKPINDIR = SYSCTL1_MCLKPINDIR_MCLKPINDIR_MASK;
#endif

	return 0;
}

SYS_INIT(mimxrt685s_hifi4_init, PRE_KERNEL_1, 0);
