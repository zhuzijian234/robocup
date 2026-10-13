"""隐藏窗口检查真实Tk布局/按钮路由，不连接真实串口。"""
from pathlib import Path
import ctypes
from ctypes import wintypes
import json
import sys
import tkinter as tk
from types import SimpleNamespace
from unittest.mock import patch
import threading

ROOT = Path(__file__).resolve().parents[2]
HOST = ROOT/'robocup-新建文件夹队/robocup-新建文件夹/上位机'
sys.path.insert(0, str(HOST))
import robocup_gui as gui

root = tk.Tk()
root.withdraw()
root.attributes('-alpha', 0.0)
try:
    gui.ttk.Style(root).theme_use('vista')
    with patch.object(gui.App, 'refresh_ports', lambda self: None):
        app = gui.App(root)
    sent=[]
    session=SimpleNamespace(capture_stop=threading.Event(), closing=threading.Event(),
        request_motor=sent.append, port='FAKE', baud=115200,
        request_capture=lambda args,speed:sent.append(('capture',args,speed)))
    app.session=session;app.connected=True;app._buttons()
    app.motor_start.invoke();app.motor_stop.invoke()
    assert sent==[17,0]
    app.speed.delete(0,'end');app.speed.insert(0,'bad')
    app.motor_stop.invoke()
    assert sent[-1]==0
    app.speed.delete(0,'end');app.speed.insert(0,'17')
    app.detect_button.invoke()
    assert sent[-1][0]=='capture' and sent[-1][2]==17
    for phase in ('negotiating','capture','dump'):
        app._event('phase',phase)
        assert app.motor_stop['state']=='normal'
        app.motor_stop.invoke()
        assert sent[-1]==0
    app.phase='capture';app._buttons();app.capture_stop.invoke()
    assert session.capture_stop.is_set()
    app.capturing=False;app.reporting=True;app._buttons()
    app.motor_stop.invoke();assert sent[-1]==0
    app.reporting=False;app.capture_text.set('等待检测')
    app.progress_text.set('');app.motor_text.set('关闭指令已确认（目标0）');app._buttons()
    root.deiconify();root.update()
    boxes={}
    def inspect(widget):
        for child in widget.winfo_children():
            if isinstance(child,(tk.Button,gui.ttk.Button)):
                text=child.cget('text')
                x,y=child.winfo_rootx()-root.winfo_rootx(),child.winfo_rooty()-root.winfo_rooty()
                boxes[text]=[x,y,child.winfo_width(),child.winfo_height()]
                assert 0<=x and x+child.winfo_width()<=root.winfo_width(),text
                assert 0<=y and y+child.winfo_height()<=root.winfo_height(),text
            inspect(child)
    inspect(root)
    assert app.log_text.winfo_height()>=90
    (Path(__file__).parent/'gui_layout.json').write_text(json.dumps(boxes,ensure_ascii=False,indent=2),encoding='utf-8')
    # PrintWindow仅绘制当前透明测试窗口，不截取用户桌面。
    user32, gdi32 = ctypes.windll.user32, ctypes.windll.gdi32
    user32.GetParent.argtypes=[wintypes.HWND];user32.GetParent.restype=wintypes.HWND
    user32.GetWindowDC.argtypes=[wintypes.HWND];user32.GetWindowDC.restype=wintypes.HDC
    gdi32.CreateCompatibleDC.argtypes=[wintypes.HDC];gdi32.CreateCompatibleDC.restype=wintypes.HDC
    gdi32.CreateCompatibleBitmap.argtypes=[wintypes.HDC,ctypes.c_int,ctypes.c_int];gdi32.CreateCompatibleBitmap.restype=wintypes.HBITMAP
    gdi32.SelectObject.argtypes=[wintypes.HDC,wintypes.HANDLE];gdi32.SelectObject.restype=wintypes.HANDLE
    gdi32.GetBitmapBits.argtypes=[wintypes.HBITMAP,ctypes.c_long,ctypes.c_void_p]
    user32.PrintWindow.argtypes=[wintypes.HWND,wintypes.HDC,wintypes.UINT]
    gdi32.DeleteObject.argtypes=[wintypes.HANDLE];gdi32.DeleteDC.argtypes=[wintypes.HDC]
    user32.ReleaseDC.argtypes=[wintypes.HWND,wintypes.HDC]
    hwnd=user32.GetParent(root.winfo_id());rect=wintypes.RECT();user32.GetWindowRect(hwnd,ctypes.byref(rect))
    width,height=rect.right-rect.left,rect.bottom-rect.top
    dc=user32.GetWindowDC(hwnd);memory=gdi32.CreateCompatibleDC(dc)
    bitmap=gdi32.CreateCompatibleBitmap(dc,width,height);previous=gdi32.SelectObject(memory,bitmap)
    try:
        assert user32.PrintWindow(hwnd,memory,2)
        data=ctypes.create_string_buffer(width*height*4)
        gdi32.GetBitmapBits(bitmap,len(data),data)
        from PIL import Image
        Image.frombuffer('RGB',(width,height),data.raw,'raw','BGRX',0,1).save(Path(__file__).parent/'gui_preview.png')
    finally:
        gdi32.SelectObject(memory,previous);gdi32.DeleteObject(bitmap);gdi32.DeleteDC(memory);user32.ReleaseDC(hwnd,dc)
    print('PASS hidden Tk layout, one-click capture speed, independent start/stop, invalid speed stop, all phases and report stop')
finally:
    root.destroy()
