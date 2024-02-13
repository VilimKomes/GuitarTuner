#!/usr/bin/python3
# Moduli

import sys
import signal
import numpy as np
import pyaudio
from math import floor
from scipy.signal import find_peaks
from NotesAndFreq import note_and_freq,frequencies,freq_dif
from Tunings import NST,half_step,standard,open_D
import busio
from board import SCL, SDA
# https://pypi.org/project/oled-text/
from oled_text import OledText, Layout64, BigLine

# led
import RPi.GPIO as GPIO
# button
from gpiozero import Button

def EXIT(signal,frame):
    stream.stop_stream()
    stream.close()
    oled.layout=Layout64.layout_1big_center()
    oled.clear()
    GPIO.setmode(GPIO.BCM)
    GPIO.cleanup()
    sys.exit(0)


# button
button = Button(17)
button2 = Button(27)


# oled screen setup
i2c=busio.I2C(SCL,SDA)
oled=OledText(i2c,128,64)

oled.layout=Layout64.layout_1big_center()
oled.on_draw=lambda draw: draw.rectangle((0,0,127,63),outline=255,fill=0)

# LED's setup
"""
    .BCM uses GPIO numbering, while .BOARD uses pin numbering
"""
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(14, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(15, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(18, GPIO.OUT, initial=GPIO.LOW)


def main():

    FSAMP = 44100
    FRAME = 4096
    FRAMES_FFT = 4
    SAMPLES_FFT = FRAME * FRAMES_FFT
    hann_window = np.hanning(SAMPLES_FFT)
    num_frames = 0

    global stream

    # FFT return size
    N = int(SAMPLES_FFT)/2+1 if int(SAMPLES_FFT/2)%2==0 else int((SAMPLES_FFT+1)/2)
    dic_freq={}
    buff = np.zeros(SAMPLES_FFT, dtype=np.float32)
    freq_range = np.float32(np.arange(N)*(FSAMP/len(buff)))


    choice=[[half_step,"half_step"],[NST,"NST"],[open_D,"open_D"],[standard,"standard"]]
    what_choice=0
    i=0

    while True:
        if i>len(choice)-1:
            i=0
        oled.text(" "+choice[i][1],1)
        button.wait_for_press(timeout=0.1)
        if button.is_pressed:
            what_choice=i
            break
        button.wait_for_release()

        button2.wait_for_press(timeout=0.1)
        if button2.is_pressed:
            i=i+1
        button2.wait_for_release()



    oled.layout = {
            1: BigLine(22, 0, font="Arimo.ttf", size=13),
            2: BigLine(22, 15, font="Arimo.ttf", size=13),
            3: BigLine(40, 30, font="Arimo.ttf", size=24),
    }

    tun=' '.join(list(choice[what_choice][0].values()))

    fir_row = tun[:floor(len(tun)/2)][:-1] if tun[:floor(len(tun)/2)][-1] == ' ' else tun[:floor(len(tun)/2)]
    sec_row = tun[floor(len(tun)/2):][1:] if tun[floor(len(tun)/2):][0] == ' ' else tun[floor(len(tun)/2):]

    oled.clear()
    oled.on_draw=lambda draw: draw.rectangle((0,0,127,63),outline=255,fill=0)
    oled.text(fir_row,1)
    oled.text(sec_row,2)
    stream = pyaudio.PyAudio().open(format=pyaudio.paInt16, channels=1, rate=FSAMP, input=True, FRAMES_FFT_bufffer=FRAME)

    while True:
        buff[:-FRAME] = buff[FRAME:]
        buff[-FRAME:] = np.frombufffer(stream.read(FRAME,exception_on_overflow=False), np.int16)/32768.0
        fft = np.fft.rfft(buff*hann_window)

        pos, h = find_peaks(abs(fft), height=max(fft)/20)

        num_frames += 1
        if num_frames >= FRAMES_FFT:
            for i in range(0,len(h["peak_heights"])):
                dic_freq[freq_range[pos][i]] = h["peak_heights"][i]

            freq = sorted(dic_freq.items(), key=lambda item: item[1], reverse=True)
            freq=freq[:2]
            freq.sort()
            f1=freq[0][0]

            q=min(frequencies, key=lambda x:abs(x-f1))
            low = q - freq_dif[frequencies.index(q)-1]*0.05
            high = q + freq_dif[frequencies.index(q)]*0.05

            if f1<low:
                GPIO.output(14, GPIO.HIGH)
                GPIO.output(15, GPIO.LOW)
                GPIO.output(18, GPIO.LOW)
            elif f1>high:
                GPIO.output(14, GPIO.LOW)
                GPIO.output(15, GPIO.LOW)
                GPIO.output(18, GPIO.HIGH)
            else:
                GPIO.output(14, GPIO.LOW)
                GPIO.output(15, GPIO.HIGH)
                GPIO.output(18, GPIO.LOW)


            note = note_and_freq[format(min(frequencies,key=lambda x:abs(x-f1)),'.2f')][0]
            oled.text(note,3)
            dic_freq.clear()

if __name__ == '__main__':
    signal.signal(signal.SIGINT,EXIT)
    main()
