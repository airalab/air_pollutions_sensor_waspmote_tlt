#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import time
import threading
import ssl
from SocketServer import TCPServer, ThreadingMixIn, StreamRequestHandler
import sqlite3
import pyotp
from .waspframe_parser import waspframe_parse


class SSL_TCPServer(TCPServer):
    def __init__(self,
                 server_address,
                 RequestHandlerClass,
                 certfile,
                 keyfile,
                 ssl_version=ssl.PROTOCOL_TLSv1,
                 bind_and_activate=True):
        TCPServer.__init__(self, server_address, RequestHandlerClass, bind_and_activate)
        self.certfile = certfile
        self.keyfile = keyfile
        self.ssl_version = ssl_version

    def get_request(self):
        newsocket, fromaddr = self.socket.accept()
        connstream = ssl.wrap_socket(newsocket,
                                 server_side=True,
                                 certfile = self.certfile,
                                 keyfile = self.keyfile,
                                 ssl_version = self.ssl_version)
        return connstream, fromaddr


class SSL_ThreadingTCPServer(ThreadingMixIn, SSL_TCPServer):
    def __init__(self,
                 server_address,
                 IncomingHandlerClass,
                 certfile,
                 keyfile,
                 otp_server,
                 incoming_proc):
        SSL_TCPServer.__init__(self, server_address, IncomingHandlerClass, certfile, keyfile)
        self.otp_server = otp_server
        self.proc = incoming_proc


class IncomingHandler(StreamRequestHandler): # instantiates for every incoming message
    def handle(self):
        msg = self.connection.recv(4096)
        server_otp = unicode(self.server.otp_server.now())
        client_otp = unicode(msg[msg.find('#STR:')+len('#STR:'):msg.rfind('#BAT:')])
        if pyotp.utils.strings_equal(server_otp, client_otp): # drop if no valid password
            self.server.proc(msg)


class WaspmoteMeasurementsDatabase:
    def __init__(self, db_filename, schema_filename):
        self.db_filename = db_filename
        self.schema_filename = schema_filename
        if not os.path.exists(db_filename):
            with open(schema_filename, 'r') as schema_file, sqlite3.connect(db_filename) as conn:
                schema = schema_file.read()
                conn.executescript(schema)

    def write(self, m):
        with sqlite3.connect(self.db_filename, detect_types=sqlite3.PARSE_DECLTYPES) as conn:
            values = (m['GMT'], m['TC'], m['HUM'], m['PRES'],
                      m['CO'], m['NO'], m['SO2'], m['PM1'], m['PM2_5'], m['PM10'])
            conn.executescript("""
            INSERT INTO measurements
                (sensor_ts, temp, hum, pres, co, no, so2, pm1, pm2_5, pm10)
            VALUES
                (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            """, values)

    def read(self): # latest
        with sqlite3.connect(self.db_filename) as conn:
            cur = conn.cusor()
            cur.execute("SELECT sensor_ts, db_ts, temp, hum, pres, co, no, so2, pm1, pm2_5, pm10"
                        "FROM measurements ORDER BY rowid DESC LIMIT 1;")
            row = cur.fetchone()
            measure = dict()
            measure['SensorTimestamp'] = row[0]
            measure['DatabaseTimestamp'] = row[1]
            measure['Temperature'] = row[2]
            measure['Temperature_unit'] = 'C'
            measure['Humidity'] = row[3]
            measure['Humidity_unit'] = '%'
            measure['Pressure'] = row[4]
            measure['Pressure_unit'] = 'Pa'
            measure['CO'] = row[5]
            measure['CO_unit'] = 'ppm'
            measure['NO'] = row[6]
            measure['NO_unit'] = 'ppm'
            measure['SO2'] = row[7]
            measure['SO2_unit'] = 'ppm'
            measure['PM1'] = row[8]
            measure['PM1_unit'] = 'ug/m3'
            measure['PM2_5'] = row[9]
            measure['PM2_5_unit'] = 'ug/m3'
            measure['PM10'] = row[10]
            measure['PM10_unit'] = 'ug/m3'
            return measure


class WaspmoteGateway:
    def __init__(self, server_address, server_secrets, otp_interval, msg_proc, parse_frame=False):
        self.parse_frame = parse_frame
        self.ext_proc = msg_proc
        self.otp_server = pyotp.TOTP(server_secrets['opt_key'], otp_interval)
        self.server = SSL_ThreadingTCPServer(server_address,
                                             IncomingHandler,
                                             server_secrets['certfile'],
                                             server_secrets['keyfile'],
                                             self.otp_server,
                                             self.msg_proc)
        self.threads = list()
        self.threads.append(threading.Thread(target=self.server.serve_forever)
        for t in self.threads:
            t.daemon = True
            t.start()
        self.db = WaspmoteMeasurementsDatabase('measurements.db', 'measurements_schema.sql')

    def spin(self):
        for t in self.threads:
            t.join()
        self.shutdown()

    def shutdown(self):
        self.server.server_close()
        self.server.shutdown()

    def msg_proc(self, msg):
        msg_parsed = waspframe_parse(msg)
        self.db.write(msg_parsed)
        if self.parse_frame:
            self.ext_proc(msg_parsed)
        elif:
            self.ext_proc(msg)

    def get_last_measurement(self):
        return self.db.read()


if __name__ == '__main__':
    WaspmoteGateway().spin() #TODO: parameters
