#!/usr/bin/env python
#codind:utf-8
import wx
import rospy

from std_msgs.msg import String


global pub,pub1
pub = rospy.Publisher("control_signal", String, queue_size=10)
pub1 = rospy.Publisher("continual_signal",String, queue_size=10)
NAME = 'control_node'
rospy.init_node(NAME, anonymous=True)
def next_signal(event):
    str = "hello mf"
    pub.publish(str)

def continual_signal(event):
    str = "hello mf"
    pub1.publish(str);


def main():
    app = wx.App()
    win = wx.Frame(None, title = "control", size = (400, 400))
    bkg = wx.Panel(win)
    controlButton= wx.Button(bkg, label = 'Next Frame')
    controlButton.Bind(wx.EVT_BUTTON, next_signal)
    continalButton= wx.Button(bkg, label = 'Continual')
    continalButton.Bind(wx.EVT_BUTTON, continual_signal)
#    exitlabelButton = wx.Button(bkg, label = 'Finish This Frame')

    hbox = wx.BoxSizer()
    hbox.Add(controlButton, proportion = 0, flag = wx.LEFT, border = 5)
    hbox.Add(continalButton, proportion=0, flag=wx.LEFT, border=5)
#    hbox.Add(exitlabelButton, proportion=0, flag=wx.LEFT, border=5)

    vbox = wx.BoxSizer(wx.VERTICAL)
    vbox.Add(hbox, proportion = 0, flag = wx.EXPAND | wx.ALL, border = 5)

    bkg.SetSizer(vbox)

    controlButton.Bind(wx.EVT_BUTTON, next_signal)

    win.Show()
    app.MainLoop()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass