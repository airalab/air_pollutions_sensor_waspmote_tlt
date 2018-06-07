# -*- coding: utf-8 -*-

from air_pollutions_sensor_waspmote_tlt.waspmote_gateway import WaspmoteGateway
from flask import Flask, request
from flask_restful import Resource, Api
from sqlalchemy import create_engine
from json import dumps
import argparse

GATEWAY_OTP_INTERVAL_DEFAULT = 3600 # seconds


app = Flask(__name__)
api = Api(app)


def get_args():
    args_parser = argparse.ArgumentParser()
    args_parser.add_argument("server_address", help="API server IP:PORT")
    args_parser.add_argument("gateway_address", help="Sensor gateway IP:PORT")
    args_parser.add_argument("gateway_secrets", help="Sensor gateway secrets directory path")
    args_parser.add_argument("--otp", help="Sensor gateway one time password interval")
    # args_parser.add_argument("--db", help="Sensor gateway database path")
    args = args_parser.parse_args()
    return args
args = get_args()

gateway_address_host, gateway_address_port = args.gateway_address.split(":")
gateway_secrets = {'certfile': args.gateway_secrets + '/server.crt',
                   'keyfile':  args.gateway_secrets + '/server.key',
                   'otp_key':  args.gateway_secrets + '/otp.key'}
sensor = WaspmoteGateway((gateway_address_host, int(gateway_address_port)), gateway_secrets,
                          args.otp if args.otp else GATEWAY_OTP_INTERVAL_DEFAULT)

class Measurement(Resource):
    def get(self):
        return {'data': sensor.latest_measurement}

api.add_resource(Measurement, '/latest_measurement')

if __name__ == '__main__':
    host, port = args.server_address.split(":")
    app.run(host=host, port=port)
