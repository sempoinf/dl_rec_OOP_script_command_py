"""
 -----------------------------------------------------------------------------
 Project       : EMG-gesture-recognition
 File          : emg_plot.py
 Author        : nktsb
 Created on    : 08.05.2024
 GitHub        : https://github.com/nktsb/EMG-gesture-recognition
 -----------------------------------------------------------------------------
 Copyright (c) 2024 nktsb
 All rights reserved.
 -----------------------------------------------------------------------------
"""

import sys
import time
import os

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.axes
from matplotlib.widgets import Button
from matplotlib.widgets import CheckButtons

# color definition
black = '#1A1A1A'
gray = '#EEEEEE'

color_set = ['#fe0000', '#00f61b', '#20E1FF', '#F94EBC', '#fff801', 
             '#ffaa3b', '#fa3bff', '#c9954c', '#b3ecf1', '#b9ff66']

class Plotter:
    def __init__(self, data: dict, max_mins: list, 
                 show_legend: bool = False, title: str = None):
        """
        Инициализация Plotter для работы с data: dict.

        :param data: словарь вида {sns: {rnge: [values]}}
        :param max_mins: список [(min, max)] для установки осей.
        :param show_legend: показывать ли легенду.
        :param title: заголовок графика.
        """
        self.data = data

        # Вычисляем минимальное окно данных
        self.window_size = min(len(values) 
                               for sns_data in data.values() 
                               for values in sns_data.values())
        self.channels = [(sns, rnge) for sns in data for rnge in data[sns]]

        # Создаём графики
        self.fig, self.ax = plt.subplots()
        self.ax.set_facecolor(black)
        self.ax.set_xlim(0, self.window_size)
        self.ax.set_ylim(max_mins[0])  # Предполагается, что max_mins[0] подходит для всех графиков

        if title:
            self.ax.set_title(title, loc='center')

        self.plot_lines = []
        for i, (sns, rnge) in enumerate(self.channels):
            line, = self.ax.plot([], [], color=color_set[i % len(color_set)], 
                                 label=f"SNS {sns}, RNG {rnge}")
            self.plot_lines.append(line)

        plt.xticks([i * self.window_size / 10 for i in range(11)])
        plt.grid(True)
        plt.axhline(y=0, color=gray)

        if show_legend:
            self._add_legend()

        self.anim = None

    def _add_legend(self):
        """Добавление чекбоксов для управления видимостью графиков."""
        visibility = [True for _ in self.channels]
        rax = self.fig.add_axes([0.8, 0.5 - (0.05 * len(self.channels)) / 2, 
                                 0.15, 0.05 * len(self.channels)])
        self.check = CheckButtons(rax, [f"SNS {sns}, RNG {rnge}" 
                                        for sns, rnge in self.channels], visibility)

        for rect, color in zip(self.check.rectangles, color_set):
            rect.set_facecolor(color)

        self.check.on_clicked(self._toggle_visibility)

    def _toggle_visibility(self, label):
        """Обработчик видимости графиков."""
        for line, (sns, rnge) in zip(self.plot_lines, self.channels):
            if label == f"SNS {sns}, RNG {rnge}":
                line.set_visible(not line.get_visible())
        plt.draw()

    def upd_plot(self, frame):
        """Обновление данных на графике."""
        for line, (sns, rnge) in zip(self.plot_lines, self.channels):
            data_values = self.data[sns][rnge][:self.window_size]
            x = np.arange(len(data_values))
            line.set_data(x, data_values)

    def animate(self):
        """Анимация обновления графика."""
        self.anim = animation.FuncAnimation(
            self.fig, self.upd_plot, interval=32, frames=1)
        plt.show()

    def stop_animate(self):
        """Остановить анимацию."""
        if self.anim:
            self.anim.event_source.stop()

    def resume_animate(self):
        """Возобновить анимацию."""
        if self.anim:
            self.anim.event_source.start()

    def show_once(self):
        """Однократное отображение графика."""
        self.upd_plot(0)
        plt.show()

    def close(self):
        """Закрытие окна графика."""
        plt.close(self.fig)