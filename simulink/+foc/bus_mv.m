function mv = bus_mv(raw)
%BUS_MV Convert a 12-bit bus ADC count to millivolts.
%   Mirrors applications/main.c::foc_bus_voltage_mv and
%   foc_platform_stm32g431.c (0.0625 divider, 3.3 V reference).
%
% 12 位母线 ADC 计数 [counts] → 母线电压 [mV]。
% 12-bit bus ADC count [counts] -> bus voltage [mV].
%
% 换算依据 / Scaling: 52800 = 3.3 V / 0.0625 * 1000，即分压比 1/16 下 4095 counts 对应
% 52.8 V 满量程、约 12.9 mV/count。先乘后加 2047 再除以 4095 等于"四舍五入到最近 mV"，
% 避免整数截断带来的系统性偏低。量程 0..52800 mV 远大于平台 7..18 V 的保护窗口，因此
% 窗口判据不会因为换算饱和而失效。
% 52800 = 3.3 V / 0.0625 * 1000, i.e. a 1/16 divider gives 52.8 V full scale over 4095
% counts, about 12.9 mV/count. Multiplying first, adding 2047 and dividing by 4095 rounds to
% the nearest mV instead of truncating, which would bias the reading low. The 0..52800 mV range
% is far wider than the 7..18 V protection window, so the window test cannot be defeated by
% converter saturation.
mv = (double(raw) * 52800 + 2047) / 4095;
end
