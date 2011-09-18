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
from StringIO import StringIO


import ecto

import os
import sys
from BaseHTTPServer import HTTPServer
from SimpleHTTPServer import SimpleHTTPRequestHandler
from multiprocessing import Process, Pipe, Queue, current_process, freeze_support
from threading import Thread
#http://docs.python.org/library/multiprocessing.html

HTML_TEXT = '''
<html>
<head><head>
<body>
<table border='1'>
<tr>
    <td><img src='%s' width='320' height='240' />,<img src='%s',width='320' height='240'/></td>
</tr>
<tr>
    <td><img src='%s' width='320' height='240' />,<img src='%s' width='320' height='240'/></td>
</tr>
</table>
</body>
</html>
''' % ('/stream', '/stream', '/stream', '/stream')

HTML_TEXT = '''
<html>
<head><head>
<body>
<img src='%s' width='320' height='240' />
</body>
</html>
''' % '/stream'

_jpegstreamers = {}
def note(format, *args):
    sys.stderr.write('[%s]\t%s\n' % (current_process().name, format % args))

def serve_stream(streamer, conn):

    note('stream starting.')
    streamer.send_response(200)
    streamer.send_header('Connection', 'close')
    streamer.send_header('Max-Age', '0')
    streamer.send_header('Expires', '0')
    streamer.send_header('Cache-Control', 'no-cache, private')
    streamer.send_header('Pragma', 'no-cache')
    streamer.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=--BOUNDARYSTRING')
    streamer.end_headers()
    count = 0
    while True:
        try:
            jpgdata = conn.recv()
            streamer.wfile.write('--BOUNDARYSTRING\r\n')
            streamer.send_header('Content-type', 'image/jpeg')
            streamer.send_header('Content-Length', str(len(jpgdata.getvalue())))
            streamer.end_headers()
            streamer.wfile.write(jpgdata.getvalue() + '\r\n')
            count = count + 1
        except socket.error, e:
            print >> sys.stderr, e
            break
        except IOError, e:
            print >> sys.stderr, e
            break
        except Exception, e:
            print >> sys.stderr, e
            break
    note('stream ending.')

class JpegStreamHandler(SimpleHTTPRequestHandler):
    '''
    The JpegStreamHandler handles requests to the threaded HTTP server.
    Once initialized, any request to this port will receive a multipart/replace
    jpeg.
    '''
    buffers = {}
    cvs = {}
    
    def start_thread(self, id):
        
    
    def do_GET(self):
        global _jpegstreamers
        global HTML_TEXT

        print 'Get Request.'
        if (self.path == '/' or not self.path):
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            self.wfile.write(HTML_TEXT)
            return
        elif (self.path == '/stream'):

            (host, port) = self.server.socket.getsockname()[:2]
            conn = _jpegstreamers[port].conn
            Thread(target=serve_stream, args=(self, stream,)).start() #spawn long running process for streaming.

    def serve_stream(self, id):
        note('stream starting.')
        self.send_response(200)
        self.send_header('Connection', 'close')
        self.send_header('Max-Age', '0')
        self.send_header('Expires', '0')
        self.send_header('Cache-Control', 'no-cache, private')
        self.send_header('Pragma', 'no-cache')
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=--BOUNDARYSTRING')
        self.end_headers()
        count = 0
        while True:
            try:
                jpgdata = self.get_latest(id)
                self.wfile.write('--BOUNDARYSTRING\r\n')
                self.send_header('Content-type', 'image/jpeg')
                self.send_header('Content-Length', str(len(jpgdata.getvalue())))
                self.end_headers()
                self.wfile.write(jpgdata.getvalue() + '\r\n')
                count = count + 1
            except socket.error, e:
                print >> sys.stderr, e
                break
            except IOError, e:
                print >> sys.stderr, e
                break
            except Exception, e:
                print >> sys.stderr, e
                break
        note('stream ending.')

    def get_latest(self, id):
        cv = self.cvs[id]
        # Consume one item
        cv.acquire()
        while not an_item_is_available():
            cv.wait()
        buffer = buffers[id]
        cv.release()
        return buffer

    def frame_grab(self, conn):
        while True:
            try:
                jpgdata = conn.recv()
            except IOError, e:
                sys.stderr.write(e)
                break
            except Exception, e:
                sys.stderr.write(e)
                break

class JpegTCPServer(SocketServer.ThreadingMixIn, SocketServer.TCPServer):
    allow_reuse_address = True


#factory class for jpegtcpservers
class JpegStreamer():
    server = ''
    host = ''
    port = ''
    sleeptime = ''
    framebuffer = ''
    counter = 0
    refreshtime = 0
    conn = None

    def __init__(self, hostandport=8080, st=0.001, conn=None):
        global _jpegstreamers
        if (type(hostandport) == int):
            self.port = hostandport
            self.host = 'localhost'
        elif (type(hostandport) == str and re.search(':', hostandport)):
            (self.host, self.port) = hostandport.split(':')
            self.port = int(self.port)
        elif (type(hostandport) == tuple):
            (self.host, self.port) = hostandport

        self.conn = conn
        self.sleeptime = st
        self.server = JpegTCPServer((self.host, self.port), JpegStreamHandler)
        _jpegstreamers[self.port] = self

    def serve_forever(self):
        self.server.serve_forever()
    def url(self):
        '''
        Returns the JpegStreams Webbrowser-appropriate URL, if not provided in the constructor, it defaults to 'http://localhost:8080'
        '''
        return 'http://' + self.host + ':' + str(self.port) + '/'


    def streamUrl(self):
        '''
        Returns the URL of the MJPEG stream. If host and port are not set in the constructor, defaults to 'http://localhost:8080/stream/'
        '''
        return self.url() + 'stream'



class ImageUpdator(ecto.Module):
    ''' A python module that does not much.'''
    def __init__(self, *args, **kwargs):
        ecto.Module.__init__(self, **kwargs)

    @staticmethod
    def declare_params(params):
        params.declare('conn', 'A pipe connection.', None)

    @staticmethod
    def declare_io(params, inputs, outputs):
        inputs.declare('ostream', 'An ecto.ostream object', None)

    def configure(self, params):
        pass

    def process(self, inputs, outputs):
        import cStringIO
        try:
            file = inputs.ostream.file
            self.params.conn.send(file)
        except Exception, e:
            print e
        return 0

def ecto_process(conn):
    import ecto
    from ecto_opencv.highgui import VideoCapture, imshow, FPSDrawer, ImageJpgWriter
    updator = ImageUpdator(conn=conn)
    video_cap = VideoCapture(video_device=0, width=1000, height=1000)
    fps = FPSDrawer()
    file = StringIO()
    writer = ImageJpgWriter(file=ecto.ostream(file))
    plasm = ecto.Plasm()
    plasm.connect(video_cap['image'] >> fps['image'],
                  fps['image'] >> writer['image'],
                  writer['file'] >> updator['ostream']
                  )
    sched = ecto.schedulers.Singlethreaded(plasm)
    sched.execute()

def serv_process(conn):
    server = JpegStreamer(hostandport=9892, st=0.001, conn=conn)
    server.serve_forever()
class Runner(object):
    def __init__(self):
        p1, p2 = Pipe()
        self.ecto_proc = Process(target=ecto_process, args=(p1,))
        self.serv_proc = Process(target=serv_process, args=(p2,))
        self.ecto_proc.start()
        self.serv_proc.start()
        self.ecto_proc.join()
        self.serv_proc.join()
    def __del__(self):
        self.ecto_proc.terminate()
        self.serv_proc.terminate()
if __name__ == '__main__':
    freeze_support()
    Runner()
