# weather-cli

Arduino NANO based web client which is gathering sensor info and sends it back to server.

Sensors used:

  DHT11
  BMP180
  DS1820


https://github.com/brbrr/weather-api

## Power consumption tests

I've use same module with 5000mAh power bank fully charged.

1. Not using any power saving modes:   [6df3742](https://github.com/brbrr/weather-cli/commit/6df3742f41c5cc80f9b368d7e9c2cdf058c9f164), module works for ~11 hours
2. Using `Sleep_n0m1` library for arduino sleep mode. In that point module was reconnecting to WIFI every cycle (Which could be not power safe)  [822b49c](https://github.com/brbrr/weather-cli/commit/822b49c3584497b98007a4dd749d5a92f4634247), works for ~13 hours
3. Include ESP deep sleep timeout, but with same config as 2. [74eb33c](https://github.com/brbrr/weather-cli/commit/74eb33c4d9fc28443b58a02ac0f0e98346209197), tests are pending
