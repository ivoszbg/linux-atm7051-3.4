#ifndef __ARM_ARCH_THERMAL_H__
#define __ARM_ARCH_THERMAL_H__

#include <linux/cpu_cooling.h>
struct leopard_tmu_platform_data {
	u8 threshold;
	u8 trigger_levels[4];
	u8 boost_trigger_levels[4];
	bool trigger_level0_en;
	bool trigger_level1_en;
	bool trigger_level2_en;
	bool trigger_level3_en;

	u8 gain;
	u8 reference_voltage;
	u8 noise_cancel_mode;
	u32 efuse_value;

	struct freq_clip_table freq_tab[8];
	struct freq_clip_table boost_freq_tab[8];
	int size[THERMAL_TRIP_CRITICAL + 1];
	unsigned int freq_tab_count;
};
#endif /* end of __ARM_ARCH_THERMAL_H__ */