#! /usr/bin/env python

"""
usage: %(progname)s [args]
"""

import roslib; roslib.load_manifest('pr2_network_management')

import rospy

import os, sys, string, time, socket, struct, re, StringIO
import urllib, urllib2, httplib
import base64
import pycurl
import subprocess
import getpass
import csv

class DDWRT:
  def __init__(self, hostname, username, password):
    self.hostname = hostname
    self.username = username
    self.password = password
    self.passman = urllib2.HTTPPasswordMgrWithDefaultRealm()

  def login(self, password = None):
    if password != None:
      self.password = password

    socket.setdefaulttimeout(5)
    auth_handler = urllib2.HTTPBasicAuthHandler(self.passman)
    opener = urllib2.build_opener(auth_handler)
    urllib2.install_opener(opener)
    
    self.passman.add_password(realm=None,
                              uri="http://%s/"%(self.hostname),
                              user=self.username,
                              passwd=self.password)

    try:
      lines = urllib2.urlopen("http://%s/Upgrade.asp"%(self.hostname)).readlines();
    except urllib2.HTTPError:
      rospy.logerror("Invalid password")
      return False
    except RuntimeError:
      return False

    # If we got the change-password prompt, set it to default
    if True in ["""<form name="changepassword" action="apply.cgi" method="post">""" in l for l in lines]:
      lines = urllib2.urlopen("http://%s/apply.cgi"%(self.hostname),data="submit_button=index&submit_type=changepass&next_page=Info.htm&change_action=gozila_cgi&action=Apply&http_username=root&http_passwd=%s&http_passwdConfirm=%s"%(self.password,self.password)).readlines();
      return self.login()

    return True


  def apply(self, args):
    url = "http://%s/apply.cgi" % self.hostname

    # Find the authorization explicitly
    auth = self.passman.find_user_password(None,url)
    base64string = base64.encodestring("%s:%s" % auth)[:-1]

    # We set the authorization directly since otherwise urllib2 actually crashes
    # (apply.cgi has poor handling of failed authorization)
    req = urllib2.Request(url)
    req.add_header("Authorization", "Basic %s" % base64string)

    if args:
      req.data = urllib.urlencode(args)
      try:
        fp = urllib2.urlopen(req)
        body = fp.read()
        fp.close()
      except httplib.BadStatusLine, e:
        print >> sys.stderr, "Failure to authenticate when applying sttings"

  def get_config(self, page):
      url = "http://%s/%s" % (self.hostname, page)

      # Find the authorization explicitly
      auth = self.passman.find_user_password(None,url)
      base64string = base64.encodestring("%s:%s" % auth)[:-1]

      req = urllib2.Request(url)
      req.add_header("Authorization", "Basic %s" % base64string)

      fp = urllib2.urlopen(req)
      text = fp.read()
      fp.close()

      # ICK. HTML+javascript parser to parse out the configuration
      tags = re.split(r'(<[^>]+>)', text)

      conf = {}

      name = ""
      name_re = re.compile(r'name=\\?"([^"]+?)\\?"')
      value_re = re.compile(r'value=\\?"([^"]+?)\\?"')
      class_re = re.compile(r'class=\\?"([^"]+?)\\?"')

      selected_re = re.compile(r'selected=\\?"[^"]+?\\?"')
      checked_re = re.compile(r'checked=\\?"[^"]+?\\?"')
      for t in tags:
         m = name_re.search(t)
         if m != None:
            name = m.group(1)
            m = class_re.search(t)
            if m != None and m.group(1) == 'spaceradio':
               m = checked_re.search(t)
               if m != None:
                  m = value_re.search(t)
                  if m != None:
                     conf[name] = m.group(1)
            else:
               m = value_re.search(t)
               if m != None:
                  conf[name] = m.group(1)
         if selected_re.search(t) != None:
            m = value_re.search(t)
            conf[name] = m.group(1)
      return conf

  def waitForBoot(self, newip=None, password=None):

    oldip = self.hostname

    if newip != None:
      self.hostname = newip
    else:
      newip = self.hostname

    # Use ip route to determine if we are likely to still be able to find the router
    if1 = "Unknown1"
    runip = subprocess.Popen(["ip", "route", "get", oldip], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    (o,e) = runip.communicate()
    res = runip.wait()

    m = re.search(r"dev (\w+)", o)
    if m is not None:
      if1 = m.group(1)

    if2 = "Unknown2"
    runip = subprocess.Popen(["ip", "route", "get", newip], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    (o,e) = runip.communicate()
    res = runip.wait()

    m = re.search(r"dev (\w+)", o)
    if m is not None:
      if2 = m.group(1)

    if (if1 != if2):
      print >> sys.stderr, "The configuration of the device has likely changed."
      print >> sys.stderr, ""
      print >> sys.stderr, "old ip %s: \t routed via '%s'"%(oldip, if1)
      print >> sys.stderr, "new ip %s: \t routed via '%s'"%(newip, if2)
      print >> sys.stderr, ""
      print >> sys.stderr, "You probably want to add an ethernet alias such as: "
      print >> sys.stderr, ""
      print >> sys.stderr, "\tsudo ifconfig %s:1 %s.X"%(if1,".".join(newip.split(".")[0:3]))
      print >> sys.stderr, ""
      print >> sys.stderr, "in order to talk to the router."
  
    fw = "Unknown"
    print "Waiting for router to boot..."

    # Give the router a little time to start the reboot
    time.sleep(5)

    found = False
    for i in xrange(100):
      try:
        if self.login(password):
          fw = self.getFirmwareVersion()
          found = True
          break
      except urllib2.HTTPError:
        time.sleep(1)
      except urllib2.URLError:
        time.sleep(1)
      except urllib2.httplib.BadStatusLine:
        time.sleep(1)
    

    if found:
      print "Router has resumed successfully on: %s with firmare: '%s'"%(newip,fw)
    else:
      print >> sys.stderr, "Router has not resumed successfully.  Please go to http://pr.willowgarage.com/wiki/PR2/Admin/Troubleshooting for information on debugging."
