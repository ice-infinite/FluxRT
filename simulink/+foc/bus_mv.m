function mv = bus_mv(raw)
%BUS_MV Convert a 12-bit bus ADC count to millivolts.
%   Mirrors applications/main.c::foc_bus_voltage_mv and
%   foc_platform_stm32g431.c (0.0625 divider, 3.3 V reference).
mv = (double(raw) * 52800 + 2047) / 4095;
end
