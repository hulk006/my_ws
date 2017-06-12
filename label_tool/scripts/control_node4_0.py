#!/usr/bin/env python
#codind:utf-8
import roslib
import wx
import time
import rospy
import sys
from std_msgs.msg import Int16
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import String


class ControlApp(wx.App):
    def OnInit(self):
        self.frame = wx.Frame(None,title = "control",size = (555,500))
        self.panel = ControlPanel(self.frame)
        self.frame.Show(True)
        return True

class ControlPanel(wx.Panel):
    def __init__(self,parent):
        wx.Panel.__init__(self,parent)
        self.control_pub = rospy.Publisher("control_signal", Int16, queue_size=1)
        self.label_signal = rospy.Publisher("label_signal", String, queue_size=1)
        self.trackText = wx.StaticText(self, label="Track", pos=(40, 15), size=(290, 60))
        self.segText = wx.StaticText(self, label="Seg", pos=(190, 15), size=(290, 60))

        self.trackShow = wx.TextCtrl(self, pos=(40, 35), size=(100, 20),
                                     style=wx.TE_AUTO_SCROLL)  # style=wx.TE_MULTILINE | wx.TE_READONLY
        self.segShow = wx.TextCtrl(self, pos=(190, 35), size=(100, 20), style=wx.TE_AUTO_SCROLL)

        self.radioList = ['Car', 'Truck', 'Bicycle', 'Pedestrian', 'Background']
        self.rb = wx.RadioBox(self, label="Label", pos=(20, 60), choices=self.radioList, majorDimension=3,
                              style=wx.RA_SPECIFY_COLS)
        self.rb.Bind(wx.EVT_RADIOBOX, self.choiceEvent)

        # self.nextLabelButton = wx.Button(self, label='Next Label', pos=(5, 140), size=(290, 60))
        # self.nextFrameButton = wx.Button(self, label='Next Frame', pos=(5, 220), size=(290, 60))
        self.lastSegButton = wx.Button(self, label='Last Seg', pos=(5, 170), size=(135, 60))
        self.deleteSegButton = wx.Button(self, label='Delete Seg', pos=(310, 170), size=(135, 60))
        self.repalyButton = wx.Button(self, label='Replay', pos=(310, 240), size=(135, 60))
        self.saveButton = wx.Button(self, label='Save', pos=(310, 310), size=(135, 60))
        self.nextSegButton = wx.Button(self, label='Next Seg', pos=(150, 170), size=(135, 60))
        self.palyButton = wx.Button(self, label='Play', pos=(5, 240), size=(280, 60))
        self.labelButton = wx.Button(self, label='Label', pos=(5, 310), size=(280, 60))
        self.nextTrackButton = wx.Button(self, label='Next Track', pos=(5, 390), size=(135, 60))
        self.lastTrackButton = wx.Button(self, label='Last Track', pos=(150, 390), size=(135, 60))
        self.deleteTrackButton = wx.Button(self, label='Delete Track', pos=(310, 390), size=(135, 60))

        self.saveButton.Bind(wx.EVT_BUTTON, self.saveEv)
        self.repalyButton.Bind(wx.EVT_BUTTON, self.replayEv)
        self.palyButton.Bind(wx.EVT_BUTTON, self.playEv)
        self.deleteSegButton.Bind(wx.EVT_BUTTON, self.deleteSegEv)
        self.labelButton.Bind(wx.EVT_BUTTON, self.labelEv)
        self.lastSegButton.Bind(wx.EVT_BUTTON, self.lastSegEv)
        self.nextSegButton.Bind(wx.EVT_BUTTON, self.nextSegEv)
        self.nextTrackButton.Bind(wx.EVT_BUTTON, self.nextTrackEv)
        self.lastTrackButton.Bind(wx.EVT_BUTTON, self.lastTrackEv)
        self.deleteTrackButton.Bind(wx.EVT_BUTTON, self.deleteTrackEv)
        self.labelc = "car"

    def deleteTrackEv(self,event):
        self.control_pub.publish(data = 10)

    def lastTrackEv(self,event):
        self.control_pub.publish(data = 9)

    def saveEv(self,event):
        self.control_pub.publish(data = 8)

    def replayEv(self,event):
        self.control_pub.publish(data = 7)

    def playEv(self,event):
        self.control_pub.publish(data = 6)

    def deleteSegEv(self, event):
        self.control_pub.publish(data = 5)

    def labelEv(self, event):
        self.control_pub.publish(data = 4)
        self.label_signal.publish(data = self.labelc)

    def lastSegEv(self, event):
        self.control_pub.publish(data = 3)

    def nextSegEv(self, event):
        self.control_pub.publish(data = 2)

    def nextTrackEv(self, event):
        self.control_pub.publish(data = 1)

    def choiceEvent(self,event):
        self.cc = event.GetInt()
        if self.cc == 0:
            self.labelc = "car"
        elif self.cc == 1:
            self.labelc = "truck"
        elif self.cc == 2:
            self.labelc = "bicycle"
        elif self.cc == 3:
            self.labelc = "pedestrian"
        elif self.cc == 4:
            self.labelc = "background"

    def update1(self,data):
        self.trackcount = data.data[0]
        self.tracknum = data.data[1]
        self.trackShow.Clear()
        self.trackShow.AppendText('%d' % self.trackcount)
        self.trackShow.AppendText('/')
        self.trackShow.AppendText('%d' % self.tracknum)

    def update2(self,data):
        self.segcount = data.data[0]
        self.segnum = data.data[1]
        self.segShow.Clear()
        self.segShow.AppendText('%d' % self.segcount)
        self.segShow.AppendText('/')
        self.segShow.AppendText('%d' % self.segnum)




def trackCallback(data):
    wx.CallAfter(wx.GetApp().panel.update1, data)

def segCallback(data):
    wx.CallAfter(wx.GetApp().panel.update2, data)



def main(argv):
    app = ControlApp()
    rospy.init_node("label_control", anonymous=True)
    rospy.Subscriber("track_num", Int16MultiArray, trackCallback)
    rospy.Subscriber("seg_num", Int16MultiArray, segCallback)
    app.MainLoop()
    return 0




if __name__ == "__main__":
    sys.exit(main(sys.argv))







