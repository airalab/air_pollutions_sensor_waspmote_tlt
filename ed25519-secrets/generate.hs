#!/usr/bin/env runhaskell
module Main where

import           Crypto.PubKey.Ed25519 (generateSecretKey, toPublic)
import           Data.ByteArray        (convert)
import           Data.ByteString       (unpack)
import           Text.Printf           (printf)

main :: IO ()
main = do
    secret <- generateSecretKey
    let public = toPublic secret
        secret_array = render (convert secret) :: String
        public_array = render (convert public) :: String
    putStrLn $ printf secrets_h secret_array public_array
  where
    secrets_h = "uint8_t signing_key[32] = { %s };\nuint8_t verifying_key[32] = { %s };"
    render key = foldl1 (printf "%s, %s") (printf "0x%02X" <$> unpack key)
