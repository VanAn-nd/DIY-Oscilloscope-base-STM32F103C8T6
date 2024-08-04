# Bluepill Oscilloscope (STM32F103C8T6-ST7789)

Mode: Normal/X-Y mode
Channel: 2-CH
Sampling rate: 1MSa/s  (STM32F303 get 10MSa/s)
Input impedance: 1M
Test signal: 1kHz / 3.3Vpp
Horizontal scale: 5us - 200ms (step by 1-2-5)
Vertical Sensitivity: 100mV - 10V (step by 1-2-5)
Waveform length: 1k - 2k (auto)
XY-Mode record length: 256pt - 2048pt (adjustable: step by 256++)
My purpose is to make a simple oscilloscope for everyone can easy to build. Because it is hard to get a high bandwidth opamp in my location. Of course you can use a greater one if you have it: AD8065,AD8061,AD8001,OPA2354,... The result good or not depending on your analog front end.
