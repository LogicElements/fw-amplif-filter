# fw-amplif-filter

Firmware for advaced amplifier with filtering features

FW includes code for ADC with external 2.5 V reference. ADC is triggered by TIM7 IRQ with sampling time 500 us (2 khz f_sample), after conversion DMA is used to store value into global buff and starts data processing, where real voltage, moving average and FIR values are calculated.
Every 2s change threshold value for external DAC depended on average.
MUX param set every 1s independend on change --> ToDo set param only when param are changed.