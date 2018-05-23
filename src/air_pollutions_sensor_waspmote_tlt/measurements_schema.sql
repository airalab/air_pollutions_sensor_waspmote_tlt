CREATE TABLE measurements (
  measurement_id     INTEGER PRIMARY KEY,
  sensor_ts          DATETIME, -- UTC
  db_ts              DATETIME DEFAULT CURRENT_TIMESTAMP, -- UTC
  temp               FLOAT, -- C
  hum                FLOAT, -- %
  pres               FLOAT  -- Pa
  co                 FLOAT, -- ppm
  no                 FLOAT, -- ppm
  so2                FLOAT, -- ppm
  pm1                FLOAT, -- ug/m3
  pm2_5              FLOAT, -- ug/m3
  pm10               FLOAT -- ug/m3
);
