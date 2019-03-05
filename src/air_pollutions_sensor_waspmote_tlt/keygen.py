import ed25519
from textwrap import wrap


def key_to_hex(key):
    key = key.to_ascii(encoding='hex')
    key = key.decode().upper()
    key = wrap(key, 2)
    key = list(map(lambda x: '0x' + x, key))
    return key

def hexes_list_key_to_string_array(key: list) -> str:
    key = ', '.join(key)
    return '{ ' + key + ' }'


print('1. generating key pair')
signing_key, verifying_key = ed25519.create_keypair()

print('2. writing key pair in bytes form')
open('secret-signing-key.bytes', 'wb').write(signing_key.to_bytes())
open('public-verifying-key.bytes', 'wb').write(verifying_key.to_bytes())

print('2. writing key pair in ascii')
sk = hexes_list_key_to_string_array(key_to_hex(signing_key))
vk = hexes_list_key_to_string_array(key_to_hex(verifying_key))
open('secret-signing-key.ascii', 'w').write(sk)
open('public-verifying-key.ascii', 'w').write(vk)
