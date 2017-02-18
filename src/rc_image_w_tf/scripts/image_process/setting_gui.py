#!/usr/bin/env python
# -*- coding: utf-8 -*-

from PyQt4.QtGui import *
from PyQt4.QtCore import *
from collections import OrderedDict
from param_server import ParamServer


class SliderItem(QWidget):

    def __init__(self, name, init_value, range_low, range_high, parent=None):
        super(SliderItem, self).__init__()
        self.name = name
        self.init_ui(name, init_value, range_low, range_high)

    def init_ui(self, name, init_value, range_low, range_high):
        # floatの場合、スライドの値1で、反映値0.025とする
        self.dpi = 1
        if isinstance(init_value, float):
            self.dpi = 40

        self.label = QLabel(ParamServer.get_param_name(name))
        self.slider = QSlider(Qt.Horizontal)  # スライダの向き
        self.slider.setRange(range_low * self.dpi, range_high * self.dpi)  # スライダの範囲
        self.slider.setValue(init_value * self.dpi)
        self.slider.setTickPosition(QSlider.TicksAbove)  # メモリの位置
        self.connect(self.slider, SIGNAL('valueChanged(int)'), self.callback)
        self.textbox = QLineEdit()
        self.textbox.setText(str(init_value))

        layout = QHBoxLayout()
        layout.setMargin(0)
        layout.addWidget(self.label, 3)
        layout.addWidget(self.slider, 6)
        layout.addWidget(self.textbox, 1)
        self.setLayout(layout)

    def callback(self):
        key = self.name
        if self.dpi == 1:
            value = self.slider.value()
        else:
            value = (float)(self.slider.value()) / self.dpi
        self.textbox.setText(str(value))
        ParamServer.set_value(key, value)


class CheckboxItem(QWidget):

    def __init__(self, name, init_value, parent=None):
        super(CheckboxItem, self).__init__()
        self.name = name
        self.init_ui(name, init_value)

    def init_ui(self, name, init_value):
        self.label = QLabel(ParamServer.get_param_name(name))
        self.checkbox = QCheckBox()
        self.checkbox.setChecked(ParamServer.get_value(name))
        self.connect(self.checkbox, SIGNAL('stateChanged(int)'), self.callback)

        layout = QHBoxLayout()
        layout.setMargin(0)
        layout.addWidget(self.label, 3)
        layout.addWidget(self.checkbox, 7)
        self.setLayout(layout)

    def callback(self):
        key = self.name
        value = self.checkbox.isChecked()
        ParamServer.set_value(key, value)


class SettingPanel(QWidget):

    def __init__(self, func_name, parent=None):
        super(SettingPanel, self).__init__()
        self.name = func_name
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout()
        # layout.setMargin(0)
        layout.setAlignment(Qt.AlignTop)

        # func_name.から始まるitemの設定項目を作成
        for key in ParamServer.get_all_params().keys():
            key_func = ParamServer.get_func_name(key)
            if (key_func == self.name):
                # typeごとに異なるwidgetを作成
                if (ParamServer.get_type(key) == ParamServer.TYPE_BOOL):
                    widget = CheckboxItem(key, ParamServer.get_value(key))
                elif (ParamServer.get_type(key) == ParamServer.TYPE_LINEAR):
                    widget = SliderItem(key,
                                        ParamServer.get_value(key),
                                        ParamServer.get_lower(key),
                                        ParamServer.get_upper(key))
                layout.addWidget(widget)

        self.setLayout(layout)


class SettingWindow(QMainWindow):

    def __init__(self):
        super(SettingWindow, self).__init__()
        self.init_ui()

    def init_ui(self):
        self.w = QWidget()
        self.w.resize(500, 150)
        self.w.setWindowTitle('Parameter')

        # tabにするwidget作成
        tabs_dict = OrderedDict()
        for key in ParamServer.get_all_params().keys():   # 'Func.Param'のkey名のOrderedDict
            key_func = ParamServer.get_func_name(key)

            # 新しいfuncが見つかったとき、widget作成
            if key_func not in tabs_dict:
                setting_widget = SettingPanel(key_func)
                tabs_dict[key_func] = setting_widget

        tab = QTabWidget()

        for key in tabs_dict.keys():
            tab.addTab(tabs_dict[key], key)

        w_layout = QHBoxLayout()
        w_layout.addWidget(tab)

        self.w.setLayout(w_layout)
        self.w.show()
