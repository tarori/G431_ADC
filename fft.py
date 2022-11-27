#!/usr/bin/env python3
import matplotlib.pyplot as plt
import csv
import pandas as pd
import numpy as np
import bottleneck as bn
from scipy import signal
import sys

decim_ratio = 1
Fs = 1e+6 / decim_ratio
#Fs = 8e+6
dt = 1 / Fs
Vref = 5
Total_bits = 65536 * decim_ratio

plot_each = False

if len(sys.argv) < 2:
    print("ファイルを指定してにょ")
    exit(1)

if len(sys.argv) == 2:
    plot_each = True

voltage_list = []
fft_abs_list = []
fft_dB_list = []
for i in range(1, len(sys.argv)):
    file_name = sys.argv[i]
    data_raw = pd.read_csv(file_name, names=['value'])
    #data_raw = data_raw['value'].str.strip("V")
    voltage = data_raw['value'] * Vref / Total_bits
    voltage = voltage.astype(float)
    N = len(voltage)
    voltage_list.append(voltage)
    times_ms = np.linspace(0, 1000*N*dt, N, endpoint=False)

    print('len:', N)
    print('NaN:',voltage.isna().sum())

    if (plot_each):
        plt.plot(times_ms, voltage)
        plt.xlabel('t (ms)')
        plt.ylabel('Signal (V)')
        plt.show()
    

    window = signal.hann(N)
    window_factor = 1/(sum(window)/N)

    voltage_ac = voltage - voltage.mean()
    voltage_ac = voltage_ac * window

    fft_complex = window_factor * np.fft.fft(voltage_ac)
    freq = np.fft.fftfreq(N, d=dt)

    fft_abs = np.abs(fft_complex / (N/2))
    fft_abs_list.append(fft_abs)
    fft_dB = 20 * np.log10(fft_abs)
    fft_dB_list.append(fft_dB)

    freq_kHz = freq * 1.0e-3
    Fs_kHz = Fs * 1.0e-3

    noise_uV = fft_abs / np.sqrt(freq[1] - freq[0]) / np.sqrt(2) * 1.0e+6
    noise_uV = bn.move_mean(noise_uV, window=64)

    if (plot_each):
        plt.plot(freq[1:int(N/2)], fft_dB[1:int(N/2)])
        plt.xlabel('f (Hz)')
        plt.ylabel('Signal (dBV)')
        plt.xscale('log')
        plt.yscale('linear')
        plt.xlim([Fs/N*8, Fs/2])
        plt.ylim([-160, 10])
        plt.grid(which="major", color="gray", linestyle="solid")
        plt.show()

        plt.plot(freq[1:int(N/2)], fft_dB[1:int(N/2)])
        plt.xlabel('f (Hz)')
        plt.ylabel('Signal (dBV)')
        plt.xscale('linear')
        plt.yscale('linear')
        plt.xticks(np.arange(0, Fs/2+1, Fs/8))
        #plt.xticks(np.arange(0, Fs/2+1, 1000))
        plt.ylim([-160, 10])
        plt.grid(which="major", color="gray", linestyle="solid")
        plt.show()

        plt.plot(freq[1:int(N/2)], noise_uV[1:int(N/2)])
        plt.xlabel('f Hz)')
        plt.ylabel('Signal (uV/√Hz)')
        plt.xscale('log')
        plt.yscale('log')
        plt.xlim([Fs/N*8, Fs/2])
        plt.ylim([0.01, 1.0e+4])
        plt.grid(which="major", color="gray", linestyle="solid")
        plt.show()

print("Total file:", len(fft_abs_list))
if (len(fft_abs_list) < 2):
    exit(0)

voltage_list = np.array(voltage_list)
fft_abs_list = np.array(fft_abs_list)
fft_dB_list = np.array(fft_dB_list)

fft_abs_avg = np.sqrt(np.mean(fft_abs_list ** 2, axis=0))
fft_dB_avg = 20 * np.log10(fft_abs_avg)
noise_uV_avg = fft_abs_avg / np.sqrt(freq[1] - freq[0]) / np.sqrt(2) * 1.0e+6

plt.plot(freq[1:int(N/2)], fft_dB_avg[1:int(N/2)])
plt.xlabel('f (Hz)')
plt.ylabel('Signal (dBV)')
plt.xscale('log')
plt.yscale('linear')
plt.xlim([Fs/N*8, Fs/2])
plt.ylim([-160, 10])
plt.grid(which="major", color="gray", linestyle="solid")
plt.show()

plt.plot(freq_kHz[1:int(N/2)], fft_dB_avg[1:int(N/2)])
plt.xlabel('f (kHz)')
plt.ylabel('Signal (dBV)')
plt.xscale('linear')
plt.yscale('linear')
#plt.xticks(np.arange(0, freq_kHz/2+1, Fs_kHz/8))
plt.xticks(np.arange(0, Fs_kHz/2+1, 1))
plt.ylim([-160, 10])
plt.grid(which="major", color="gray", linestyle="solid")
plt.show()
