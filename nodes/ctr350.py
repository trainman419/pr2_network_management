#! /usr/bin/env python

"""
usage: %(progname)s [args]
"""

import roslib; roslib.load_manifest('pr2_network_management')
import rospy

import os, sys, string, time
from optparse import OptionParser
import urllib, urllib2
import hashlib
import binascii
import base64
import subprocess
import tempfile
import json

import atexit
import shutil

import StringIO
import gzip

import socket

import random

import re

b64_wap = ".ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789_"
b64_reg = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/"
from_wap64 = string.maketrans(b64_wap, b64_reg)
to_wap64 = string.maketrans(b64_reg, b64_wap)

class ctr350:

   def __init__(self, ipaddr, passwd):
     self.ipaddr = ipaddr
     self.passwd = passwd

     # grab a copy of the shutil module so that we still have it during cleanup
     self.shutil = shutil

     self.tmp_dir = tempfile.mkdtemp(prefix='cradlepointjs')

     shutil.copy("%s/nodes/JSON-js/json2.js"%
           roslib.packages.get_pkg_dir('pr2_network_management'), self.tmp_dir)
   
     files = ["uigenpro.js","fixup_settings.js","createdata.js","pack_all.js"]
     libs = []
     for f in files:
       libs.append(self.tmp_dir + '/' + f)
     libs.append(self.tmp_dir + '/json2.js')
     self.libs = "\"%s\""%'","'.join(libs)

     # Create our override file
     f_override = open(self.tmp_dir+"/override.js","w")
   
     f_override.write("""
function executeActiveTags() {}
function alert(message) {print("ALERT: " + message);}
function restoreError (message) {alert(message); return 0;}
""")
   
     f_override.close()
     self.login()
     # Fetch the .js files we need from the server (unfortunately they are only available gzipped)
     for f in files:
       req = urllib2.Request("http://%s/%s"%(self.ipaddr,f))
       req.add_header('Accept-encoding','gzip')
       opener = urllib2.build_opener()
       f_in = opener.open(req)
       compresseddata = f_in.read()
       compressedstream = StringIO.StringIO(compresseddata)
       try:
         gzipper = gzip.GzipFile(fileobj=compressedstream)
         data = gzipper.read()
       except IOError:
         data = compresseddata
       f_out = open(self.tmp_dir+"/"+f,"w")
       f_out.write(data)
       f_out.close()
   
   def __del__(self):
      # remove the tmp directory so that our .js files get cleaned up
      print "ctr350 done"
      # I don't undetstand exactly why this is necessary. it's only a problem
      # when this object is deleted at program exit
      self.shutil.rmtree(self.tmp_dir)
   
   def hex_md5(self, str):
     m = hashlib.new('md5')
     m.update(str)
     return m.hexdigest()
   
   def login(self):
     socket.setdefaulttimeout(5)
   
     data64=urllib2.urlopen('http://%s/pre_login.js'%self.ipaddr).read()[6:-1].translate(from_wap64)
     data=base64.decodestring(data64)
   
     if self.passwd is None:
       arp = subprocess.Popen(["arp", "-n", self.ipaddr], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
       (o,e) = arp.communicate()
       res = arp.wait()
       mac = o.split()[8]
       self.passwd = ''.join(mac.split(':')[3:])
       rospy.loginfo("Using default mac-address-based password: %s"%self.passwd)
   
     shex = binascii.b2a_hex(data).rstrip('0').upper()
     shex = shex + '0'*(len(shex)%2)
     goodp = self.passwd + '0'*(16 - len(self.passwd))
     str = shex + goodp;
     str = str + '0'*(64 - len(str))
     salthash = shex + self.hex_md5(str)
   
     a = binascii.a2b_hex(salthash)
     if (len(a) % 3 > 0):
       a = a + '\x00'*(3 - len(a)%3)
   
     fin_passwd = base64.encodestring(a).translate(to_wap64)
   
     url = "http://%s/post_login.cgi?data=%s" %(self.ipaddr,fin_passwd)
     lines = urllib2.urlopen(url).readlines()
   
     return True in ["Basic_Wizard.html" in l for l in lines]
   
   def saveConfig(self, save_file):
     url = "http://%s/save_settings.cgi" % self.ipaddr
     urllib.urlretrieve(url, filename=save_file)
   
   def loadConfig(self, fn, name):
     conf = self.get_config(fn)
     if name != None:
        conf['wireless']['SSID'] = name
     self.put_config(conf)
   
   def changeSSID(self, name):
     config = self.get_config()
     if name != None:
        config['wireless']['SSID'] = name
     self.put_config(config)
   
   def waitForBoot(self, oldip):
     # Use ip route to determine if we are likely to still be able to find the router
     if1 = "Unknown1"
     runip = subprocess.Popen(["ip", "route", "get", oldip], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
     (o,e) = runip.communicate()
     res = runip.wait()
   
     m = re.search(r"dev (\w+)", o)
     if m is not None:
       if1 = m.group(1)
   
     if2 = "Unknown2"
     runip = subprocess.Popen(["ip", "route", "get", self.ipaddr], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
     (o,e) = runip.communicate()
     res = runip.wait()
   
     m = re.search(r"dev (\w+)", o)
     if m is not None:
       if2 = m.group(1)
   
     if (if1 != if2):
       rospy.logerr("""The configuration of the device has likely changed.
The old ip %s: \t routed via '%s'"%(oldip, if1)
The new ip %s: \t routed via '%s'"%(self.ipaddr, if2)

You probably want to add an ethernet alias such as:

\tsudo ifconfig %s:1 %s.X

in order to talk to the router."""%(if1,".".join(self.ipaddr.split(".")[0:3])))
       return False
     
     fw = "Unknown"
     rospy.loginfo("Waiting for WAP to boot...")
   
     # Give the router a little time to start the reboot
     time.sleep(5)
   
     found = False
     for i in xrange(100):
       try:
         if self.login():
           found = True
           break
         else:
           rospy.logerr("Password incorrect!")
       except urllib2.HTTPError:
         time.sleep(1)
       except urllib2.URLError:
         time.sleep(1)
   
     if found:
       rospy.loginfo("WAP has resumed successfully on: %s"%(self.ipaddr))
     else:
       rospy.logerr("Router has not resumed successfully.")
   
   def runjs(self, name):
     # Invoke our javascript and get back the data
     run = subprocess.Popen(["smjs", "-f", self.tmp_dir+"/"+name],
           stdout=subprocess.PIPE, stderr=subprocess.PIPE)
     (o,e) = run.communicate()
     res = run.wait()
     if res != 0:
       rospy.logerr("Javascript Error: %s %s"%(o, e))
       return None
     return o
   
   def get_config(self, fn=None):
     # Fetch the existing settings from server
     if fn == None:
        fn = "%s/server_conf.gws"%self.tmp_dir
        self.saveConfig(fn)
   
     # Create our main javascript file
     f_js = open(self.tmp_dir+"/unpack.js","w")
   
     f_js.write("""
//Create a window object
var window = new Object;

//Load libs from server
load(%s);

// Override some functions as necessary
load("%s/override.js");\n

// Load server config file
load("%s");\n

// Unpack into data structure
byte_array = convertFromBase64 (data);
data = null;
unpackAll();

// Print data as JSON so python can grab it
print(JSON.stringify(data))
"""%(self.libs, self.tmp_dir, fn))
     f_js.close()
   
     o = self.runjs('unpack.js')

     if o != None:
        data = json.loads(o)
        return data
     else:
         rospy.logerr("Config Unpacking failed")
         return None

   def put_config(self, config):
      
      config_s = json.dumps(config)

      f_js = open(self.tmp_dir+'/pack.js','w')
      f_js.write("""
// create a window object
var window = new Object;

// load our libraries
load(%s);

// load our overrides
load("%s/override.js");

// parse our config back into a js object
data = %s;

// Repack
byte_array_has_error = 0;
packAll();
if (byte_array_has_error) {
    print("Restored data not acceptable");
    throwError();
}

// print our packed config, ready for upload
print(convertToBase64(byte_array))
"""%(self.libs, self.tmp_dir, config_s))
      f_js.close()

      o = self.runjs('pack.js')
      if o == None:
         rospy.logerr("Config packing failed")
         return False

      new_config = o
    
      new_ip = config['lan_network_address']
      new_passwd = config['password']
      new_ssid   = config['wireless']['SSID']
      if new_config != None:
        urllib2.urlopen("http://%s/restore_settings.cgi?link=Tools_Admin.html"%(self.ipaddr),"data=%s"%new_config)
        cmd = "\x01\x00\x00"
        cmd_enc = base64.encodestring(cmd).translate(to_wap64)
        urllib2.urlopen("http://%s/cmd.cgi?data=%s"%(self.ipaddr,cmd_enc))
      else:
        rospy.logerr("Error: failure in generating config settings")
        return False
    
      self.passwd = new_passwd
      rospy.loginfo("Uploaded configuration.")
      rospy.loginfo("New ip: %s.  New password: %s.  New SSID: %s."%(new_ip, new_passwd, new_ssid))
    
      old_ip = self.ipaddr
      self.ipaddr = new_ip
      self.waitForBoot(old_ip)
      return True
   
if __name__ == '__main__':
   c = ctr350('10.68.0.250', 'willow')
   conf = c.get_config()
   c.put_config(conf)
