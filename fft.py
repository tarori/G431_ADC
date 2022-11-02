#!/usr/bin/env python3
import matplotlib.pyplot as plt
import csv
import pandas as pd
import numpy as np
import bottleneck as bn
from scipy import signal
import sys

Fs = 1e+6
#Fs = 1e+6 / 32
#Fs = 8e+6
dt = 1 / Fs
Vref = 5.0

if len(sys.argv) == 1:
    print("ファイルを指定してにょ")
    exit(1)

df = pd.read_csv(sys.argv[1], names=['value'])
#df = df['value'].str.strip("V")
df = df['value'] * Vref / 16384
df = df.astype(float)

N = len(df)
#N=65536
df = df[0:N]

plt.plot(df)
plt.xlabel('t (sample)')
plt.ylabel('Signal (V)')
plt.show()

window = signal.hann(N)
window_factor = 1/(sum(window)/N)

print('df:', df)

df = df - df.mean()
df = df * window

F = window_factor * np.fft.fft(df)
freq = np.fft.fftfreq(N, d=dt)
F = np.abs(F)
Amp = np.abs(F/(N/2))
dB = 20 * np.log10(Amp)

freq_kHz = freq * 1.0e-3
Fs_kHz = Fs * 1.0e-3

noise_uV = Amp / np.sqrt(freq[1] - freq[0]) / np.sqrt(2) * 1.0e+6
noise_uV = bn.move_mean(noise_uV, window=32)

plt.plot(freq[1:int(N/2)], dB[1:int(N/2)])
plt.xlabel('f (Hz)')
plt.ylabel('Signal (dBV)')
plt.xscale('log')
plt.yscale('linear')
plt.xlim([Fs/N*8, Fs/2])
plt.ylim([-140, 20])
plt.show()

plt.plot(freq[1:int(N/2)], dB[1:int(N/2)])
plt.xlabel('f Hz)')
plt.ylabel('Signal (dBV)')
plt.xscale('linear')
plt.yscale('linear')
plt.xticks(np.arange(0, Fs/2+1, Fs/8))
plt.ylim([-140, 20])
plt.show()

plt.plot(freq[1:int(N/2)], noise_uV[1:int(N/2)])
plt.xlabel('f Hz)')
plt.ylabel('Signal (uV/√Hz)')
plt.xscale('log')
plt.yscale('log')
plt.xlim([Fs/N*8, Fs/2])
plt.ylim([0.1, 1.0e+4])
plt.show()
