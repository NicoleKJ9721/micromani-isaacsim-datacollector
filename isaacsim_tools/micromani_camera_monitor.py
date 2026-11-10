#!/usr/bin/env python3
"""Independent three-camera monitor. Reads actual Isaac sensor RGB; never controls simulation."""
import argparse,base64,json,sys,time
from collections import deque
from PyQt5 import QtCore,QtGui,QtWidgets,QtNetwork

ROLES=[('left_wrist','左腕相机 · 模型左 / 操作者右'),('right_wrist','右腕相机 · 模型右 / 操作者左'),('global','全局相机')]
class ImageView(QtWidgets.QLabel):
    double_clicked=QtCore.pyqtSignal()
    def __init__(self):
        super().__init__('等待实时图像…');self.original=None
        self.setAlignment(QtCore.Qt.AlignCenter);self.setMinimumSize(240,180)
        self.setSizePolicy(QtWidgets.QSizePolicy.Ignored,QtWidgets.QSizePolicy.Expanding)
        self.setStyleSheet('background:#080d16;color:#8796aa;border-radius:8px;')
    def set_image(self,pix):self.original=pix;self.redraw()
    def redraw(self):
        if self.original:self.setPixmap(self.original.scaled(self.size(),QtCore.Qt.KeepAspectRatio,QtCore.Qt.SmoothTransformation))
    def resizeEvent(self,event):super().resizeEvent(event);self.redraw()
    def mouseDoubleClickEvent(self,event):self.double_clicked.emit()

class Monitor(QtWidgets.QMainWindow):
    def __init__(self,url):
        super().__init__();self.url=url;self.pending=None;self.last_seq=None;self.last_seen=0;self.frozen=False
        self.views={};self.labels={};self.samples={n:deque(maxlen=30) for n,_ in ROLES};self.stamps={};self.dialogs={};self.clock_samples=deque(maxlen=30)
        self.setWindowTitle('MicroMani · 三路相机实时监视器');self.resize(1800,640)
        self.setStyleSheet('QWidget{background:#111827;color:#e5edf7;font-size:14px} QPushButton{background:#263449;padding:8px 16px;border-radius:5px} QLabel{border:0}')
        widget=QtWidgets.QWidget();self.setCentralWidget(widget);layout=QtWidgets.QVBoxLayout(widget)
        bar=QtWidgets.QHBoxLayout();title=QtWidgets.QLabel('三路实时图像  /  640 × 480 RGB');title.setStyleSheet('font-size:22px;font-weight:600');bar.addWidget(title);bar.addStretch()
        self.pause=QtWidgets.QPushButton('暂停显示');self.pause.clicked.connect(self.toggle);bar.addWidget(self.pause);layout.addLayout(bar)
        grid=QtWidgets.QHBoxLayout()
        for name,title in ROLES:
            column=QtWidgets.QVBoxLayout();column.addWidget(QtWidgets.QLabel(title));view=ImageView();view.double_clicked.connect(lambda n=name:self.enlarge(n));column.addWidget(view,1)
            label=QtWidgets.QLabel('等待 Isaac Sim 发布画面');column.addWidget(label);grid.addLayout(column,1);self.views[name]=view;self.labels[name]=label
        layout.addLayout(grid,1);self.status=QtWidgets.QLabel('连接中…');layout.addWidget(self.status)
        self.hint=QtWidgets.QLabel('双击图像可独立放大 · 暂停显示不会暂停仿真 · 帧率为实际墙钟更新率');self.hint.setStyleSheet('color:#8fa1b8');layout.addWidget(self.hint)
        self.manager=QtNetwork.QNetworkAccessManager(self);self.timer=QtCore.QTimer(self);self.timer.timeout.connect(self.tick);self.timer.start(15)
    def toggle(self):
        self.frozen=not self.frozen;self.pause.setText('继续实时显示' if self.frozen else '暂停显示')
        if self.frozen:self.status.setText('已暂停显示；Isaac Sim 继续运行。')
    def enlarge(self,name):
        if name in self.dialogs:self.dialogs[name][0].raise_();return
        dialog=QtWidgets.QDialog(self);dialog.setAttribute(QtCore.Qt.WA_DeleteOnClose);dialog.setWindowTitle(dict(ROLES)[name]);dialog.resize(960,760)
        layout=QtWidgets.QVBoxLayout(dialog);view=ImageView();layout.addWidget(view);self.dialogs[name]=(dialog,view)
        if self.views[name].original:view.set_image(self.views[name].original)
        dialog.destroyed.connect(lambda:self.dialogs.pop(name,None));dialog.show()
    def tick(self):
        if self.frozen:return
        if self.last_seen and time.monotonic()-self.last_seen>2:self.status.setText('画面未更新：仿真可能暂停或连接中断；当前显示最后一帧。')
        if self.pending:return
        request=QtNetwork.QNetworkRequest(QtCore.QUrl(self.url));request.setAttribute(QtNetwork.QNetworkRequest.CacheLoadControlAttribute,QtNetwork.QNetworkRequest.AlwaysNetwork)
        reply=self.manager.get(request);self.pending=reply;reply.finished.connect(lambda:self.received(reply))
        QtCore.QTimer.singleShot(5000,lambda:self.timeout(reply))
    def timeout(self,reply):
        if self.pending is reply:reply.abort()
    def received(self,reply):
        self.pending=None
        try:
            if self.frozen:return
            if reply.error()!=QtNetwork.QNetworkReply.NoError:
                if not self.last_seen:self.status.setText('等待 Isaac Sim 连接／相机初始化…（自动重连）')
                elif time.monotonic()-self.last_seen>2:self.status.setText('连接暂时中断；保留最后画面并自动重连。')
                return
            data=json.loads(bytes(reply.readAll()));seq=(data['sequence'],data['published_at'])
            if seq==self.last_seq:return
            self.last_seq=seq;now=time.monotonic();self.last_seen=now
            self.clock_samples.append((now,data['simulation_time']))
            first,last=self.clock_samples[0],self.clock_samples[-1]
            rtf=(last[1]-first[1])/(last[0]-first[0]) if last[0]>first[0] else 0
            for name,_ in ROLES:
                item=data['cameras'][name];pix=QtGui.QPixmap();pix.loadFromData(base64.b64decode(item['jpeg']),'JPEG')
                if pix.isNull():raise ValueError('invalid JPEG')
                self.views[name].set_image(pix)
                if name in self.dialogs:self.dialogs[name][1].set_image(pix)
                stamp=item['render_time']
                if self.stamps.get(name)!=stamp:self.samples[name].append(now);self.stamps[name]=stamp
                times=self.samples[name];fps=(len(times)-1)/(times[-1]-times[0]) if len(times)>1 else 0
                target_rate=item.get('sample_rate_hz') or '?'
                self.labels[name].setText(f'仿真时间 {stamp:.3f} s    仿真采样 {target_rate} Hz · 实际 {fps:.1f} FPS' if times and now-times[-1]<2 else f'仿真时间 {stamp:.3f} s    此通道未更新')
            stamps=[v['render_time'] for v in data['cameras'].values()];spread=max(stamps)-min(stamps)
            age=max(0,time.time()-data['published_at'])
            if age>2:
                self.status.setText(f'画面已停留 {age:.1f} s：仿真暂停或图像源未更新。');return
            self.status.setText(f'实时连接  ·  第 {data["sequence"]} 组画面  ·  仿真速度 {rtf:.2f}×  ·  三路时间差 {spread*1000:.2f} ms  ·  发布延迟 {age*1000:.0f} ms')
        except (ValueError,KeyError,TypeError) as exc:self.status.setText('画面数据异常：'+str(exc))
        finally:reply.deleteLater()

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--port',type=int,default=49011);args=parser.parse_args()
    app=QtWidgets.QApplication(sys.argv);window=Monitor(f'http://127.0.0.1:{args.port}/frame');window.show();sys.exit(app.exec_())
