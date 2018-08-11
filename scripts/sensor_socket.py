#!/usr/bin/env python
# -*- coding: utf-8 -*-


import ed25519
import socket
import base64
import binascii
import pexpect
import argparse
from urllib.parse import urlparse
import json

IPFS_TOPIC = 'environment.airalab.org'
MSG_LENGTH = 512

count = 0
good = 0
bad = 0
corrupted = 0
message = 0

def get_args():
    args_parser = argparse.ArgumentParser()
    args_parser.add_argument("gateway_address", help="Sensor gateway IP:PORT")
    args_parser.add_argument("ipfs_http_provider", help="IPFS http API")
    args_parser.add_argument("verifying_key", help="Ed25519 verifying public key")
    args_parser.add_argument("unknown_args", nargs=argparse.REMAINDER)
    args = args_parser.parse_args()
    return args

def publish(api, topic, msg):
    msgdata = base64.b64encode(json.dumps(msg).encode('utf-8')).decode('utf-8')
    return pexpect.spawn('ipfs --api={0} pubsub pub {1} "{2}\r\n"'.format(api, topic, msgdata)).expect(pexpect.EOF)


if __name__ == '__main__':
    cli_args = get_args()
    host, port = cli_args.gateway_address.split(":")
    ipfs_api = urlparse(cli_args.ipfs_http_provider).netloc.split(':')

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_address = (host, int(port))
    sock.bind(server_address)
    sock.listen(1)

    keydata = open(cli_args.verifying_key, "rb").read()
    verifying_key = ed25519.VerifyingKey(keydata)

    while True:
        print('Waiting for a connection...')
        conn, addr = sock.accept()
        print('Connection from', addr)

        data = conn.recv(1024)
        print('Recieved {!r}'.format(data))
        count += 1

        print('Checking signature...')
        try:
            raw_msg, raw_sig = data[:-1].split(b'STR:')
        except ValueError:
            print('Corrupted message.')
            message += 1
            print('Sign: good {}, bad {}, corrupted {}. Msg: total {}, corrupted {}.'.format(good, bad, corrupted, count, message))
            continue

        msg = binascii.hexlify(raw_msg)
        sig = binascii.hexlify(raw_sig)[:128] # binary
        msg = bytearray(msg)
        msg[9] = msg[9] - 1
        msg = msg + b'0' * (MSG_LENGTH - len(msg))
        msg = binascii.a2b_hex(msg)
        print('Message: {!r}'.format(msg))
        print('Signature: {}, len: {}'.format(sig, len(sig)))

        try:
            verifying_key.verify(sig, msg, encoding='hex')
            good += 1
            print('Good signature!')
            print('Sign: good {}, bad {}, corrupted {}. Msg: total {}, corrupted {}.'.format(good, bad, corrupted, count, message))
            publish(ipfs_api, IPFS_TOPIC, msg[:len(raw_msg)].decode(errors='backslashreplace') + '#SIG:' + sig.decode(errors='backslashreplace'))

        except ed25519.BadSignatureError:
            bad += 1
            print('Bad signature!')
            print('Sign: good {}, bad {}, corrupted {}. Msg: total {}, corrupted {}.'.format(good, bad, corrupted, count, message))
        except AssertionError:
            bad += 1
            print('Corrupted signature.')
            print('Sign: good {}, bad {}, corrupted {}. Msg: total {}, corrupted {}.'.format(good, bad, corrupted, count, message))
