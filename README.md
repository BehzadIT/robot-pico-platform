# Blip

## Pico Deploy
The Pico is uploaded from the real firmware paths via the shared `Upload to Pico` MicroPython run configuration.

Uploaded firmware paths:
- `main.py`
- `src/`
- `lib/`
- `settings/`

Local-only device file:
- `settings/secrets.py`

`settings/secrets.py` must exist on your machine before upload and must define:

```python
WIFI_CREDENTIALS = {
    "ssid": "your-wifi-ssid",
    "password": "your-wifi-password",
}
```

`settings/config.py` is versioned and contains non-secret Wi-Fi/runtime settings such as timeouts and retry behavior.

Root directory purpose:
- `main.py`: Pico firmware bootstrap
- `src/`: project-owned runtime code
- `lib/`: MicroPython runtime dependencies
- `settings/`: Pico configuration package
- `docs/`: reference material
- `experiments/`: non-production exploratory scripts
