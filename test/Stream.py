#!/usr/bin/python
import os
import sys
import warnings
import time
import socket
import re
import urllib2
import types
import SocketServer
import threading
from copy import copy
from math import sqrt, atan2
from SimpleHTTPServer import SimpleHTTPRequestHandler
from types import IntType, LongType, FloatType, InstanceType
from cStringIO import StringIO


_jpegstreamers = {}
class JpegStreamHandler(SimpleHTTPRequestHandler):
    """
    The JpegStreamHandler handles requests to the threaded HTTP server.
    Once initialized, any request to this port will receive a multipart/replace
    jpeg.   
    """


    def do_GET(self):
        global _jpegstreamers

        print "got a Get"
        if (self.path == "/" or not self.path):
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            self.wfile.write("""
<html>
<head>
<style type=text/css>
    body { background-image: url(/stream); background-repeat:no-repeat; background-position:center top; background-attachment:fixed; height:100% }
</style>
</head>
<body>
&nbsp;
</body>
</html>
            """)
            return


        elif (self.path == "/stream"):
            self.send_response(200)
            self.send_header("Connection", "close")
            self.send_header("Max-Age", "0")
            self.send_header("Expires", "0")
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Pragma", "no-cache")
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=--BOUNDARYSTRING")
            self.end_headers()
            (host, port) = self.server.socket.getsockname()[:2]


            count = 0
            timeout = 0.10
            lasttimeserved = 0
            print "serving"

            while (1):
                if (_jpegstreamers[port].refreshtime > lasttimeserved or time.time() - timeout > lasttimeserved):
                    try:
                        self.wfile.write("--BOUNDARYSTRING\r\n")
                        self.send_header("Content-type", "image/jpeg")
                        self.send_header("Content-Length", str(len(_jpegstreamers[port].jpgdata.getvalue())))
                        self.end_headers()
                        self.wfile.write(_jpegstreamers[port].jpgdata.getvalue() + "\r\n")
                        lasttimeserved = time.time()
                    except socket.error, e:
                        return
                    except IOError, e:
                        return
                    count = count + 1


                time.sleep(_jpegstreamers[port].sleeptime)




class JpegTCPServer(SocketServer.ThreadingMixIn, SocketServer.TCPServer):
    allow_reuse_address = True


#factory class for jpegtcpservers
class JpegStreamer():
    """
    The JpegStreamer class allows the user to stream a jpeg encoded file
    to a HTTP port.  Any updates to the jpg file will automatically be pushed
    to the browser via multipart/replace content type.


    To initialize:
    js = JpegStreamer()


    to update:
    js.jpgdata = StringIO() 
    saveimg.getPIL().save(js.jpgdata, "jpeg") #save via PIL to a StringIO handle 
    fh.refreshtime = time.time()

    to open a browser and display:
    import webbrowser
    webbrowser.open(js.url)


    Note 3 optional parameters on the constructor:
    - port (default 8080) which sets the TCP port you need to connect to
    - sleep time (default 0.1) how often to update.  Above 1 second seems to cause dropped connections in Google chrome


    Once initialized, the buffer and sleeptime can be modified and will function properly -- port will not.
    """
    server = ""
    host = ""
    port = ""
    sleeptime = ""
    framebuffer = ""
    counter = 0
    refreshtime = 0


    def __init__(self, hostandport=8080, st=0.03):
        global _jpegstreamers
        if (type(hostandport) == int):
            self.port = hostandport
            self.host = "localhost"
        elif (type(hostandport) == str and re.search(":", hostandport)):
            (self.host, self.port) = hostandport.split(":")
            self.port = int(self.port)
        elif (type(hostandport) == tuple):
            (self.host, self.port) = hostandport


        self.sleeptime = st
        self.server = JpegTCPServer((self.host, self.port), JpegStreamHandler)
        self.server_thread = threading.Thread(target=self.server.serve_forever)
        _jpegstreamers[self.port] = self
        self.server_thread.daemon = True
        self.server_thread.start()
        self.framebuffer = self #self referential, ugh.  but gives some bkcompat


    def url(self):
        """
        Returns the JpegStreams Webbrowser-appropriate URL, if not provided in the constructor, it defaults to "http://localhost:8080"
        """
        return "http://" + self.host + ":" + str(self.port) + "/"


    def streamUrl(self):
        """
        Returns the URL of the MJPEG stream. If host and port are not set in the constructor, defaults to "http://localhost:8080/stream/"
        """
        return self.url() + "stream"
import ecto
class ImageUpdator(ecto.Module):
    """ A python module that does not much."""
    def __init__(self, *args, **kwargs):
        ecto.Module.__init__(self, **kwargs)

    @staticmethod
    def declare_params(params):
        params.declare("cell", "A cell.", None)
        params.declare("server", "Server.", None)

    @staticmethod
    def declare_io(params, inputs, outputs):
        inputs.declare("file", "A file like object", None)

    def configure(self, params):
        pass

    def process(self, inputs, outputs):
        self.params.server.jpgdata = StringIO()
        self.params.cell.params.file = ecto.ostream(self.params.server.jpgdata)
        time.sleep(0.1)
        return 0

if __name__ == '__main__':
    import ecto
    from ecto_opencv.highgui import VideoCapture, imshow, FPSDrawer, ImageJpgWriter
    js = JpegStreamer(hostandport=9892)
    js.jpgdata = StringIO()
    print js.url()
    video_cap = VideoCapture(video_device=0, width=640, height=480)
    fps = FPSDrawer()
    writer = ImageJpgWriter(file=ecto.ostream(js.jpgdata))
    plasm = ecto.Plasm()
    plasm.connect(video_cap['image'] >> fps['image'],
                  fps['image'] >> (imshow()[:], writer['image']),
                  writer['file'] >> ImageUpdator(server=js, cell=writer)[:]
                  )
    sched = ecto.schedulers.Singlethreaded(plasm)
    sched.execute()

