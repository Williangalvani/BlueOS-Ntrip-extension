# BlueOS RTK NTRIP Extension

A BlueOS Extension that provides Real-Time Kinematic (RTK) positioning capabilities by connecting to NTRIP casters and forwarding correction data to the autopilot via mavlink2rest.

## Features

- **NTRIP Client**: Connects to NTRIP casters to receive RTK correction data
- **Real-time Status**: Live monitoring of connection status and data throughput
- **Simple Configuration**: Web-based interface for easy setup of NTRIP parameters
- **mavlink2rest Integration**: Forwards RTCM correction data to the autopilot automatically
- **Persistent Settings**: Configuration is saved in a local JSON file
- **Self-contained**: Runs on any machine without special privileges

## What is RTK?

Real-Time Kinematic positioning is a satellite navigation technique that provides centimeter-level positioning accuracy by using correction data from a base station. This extension:

1. Connects to an NTRIP caster (Network Transport of RTCM via Internet Protocol)
2. Receives RTCM correction messages from a nearby base station
3. Forwards these corrections to your vehicle's GPS via mavlink2rest
4. Enables high-precision positioning for autonomous operations

## Local Development & Testing

### Quick Start

1. **Install Dependencies**:
   ```bash
   pip install -r requirements.txt
   ```

2. **Run the Extension**:
   ```bash
   cd app
   python main.py
   ```

3. **Open Web Interface**: Navigate to `http://localhost:8000`

### Command Line Options

```bash
python main.py --help
```

Available options:
- `--mavlink2rest-url`: mavlink2rest service URL (default: http://localhost:6040)
- `--host`: Host to bind to (default: 0.0.0.0)
- `--port`: Port to bind to (default: 8000)
- `--config-file`: Configuration file path (default: rtk_config.json)
- `--reload`: Enable auto-reload for development

### Example Usage

```bash
# Run with custom mavlink2rest URL
python main.py --mavlink2rest-url http://192.168.1.100:6040

# Run on different port with auto-reload
python main.py --port 9000 --reload

# Use custom config file
python main.py --config-file /path/to/my_config.json
```

## BlueOS Deployment

1. **Install the Extension** through the BlueOS Extension Manager
2. **Configure NTRIP Settings**:
   - Enter your NTRIP caster URL (e.g., `rtk2go.com:2101`)
   - Specify the mountpoint for your region
   - Add username/password if required
   - Configure mavlink2rest URL (usually `http://host.docker.internal:6040`)
   - Enable the connection
3. **Monitor Status**: View real-time connection status and data reception
4. **Verify GPS**: Check your vehicle's GPS status for RTK fix

## Configuration

The extension stores configuration in a local JSON file with the following parameters:

- **NTRIP Caster URL**: The hostname and port of your NTRIP service
- **Mountpoint**: The specific correction stream for your geographic area  
- **Username/Password**: Authentication credentials (if required)
- **mavlink2rest URL**: URL of the mavlink2rest service for sending RTCM data
- **Enable**: Toggle to start/stop the NTRIP connection

Configuration is automatically saved to `config/rtk_config.json` (or the file specified with `--config-file`).

### Docker Volume Mounting

For persistent configuration across container restarts, you can mount the config directory:

```bash
# Mount config directory for persistent settings
docker run -v ./config:/app/config blueos-rtk-extension

# Or when building/running locally
python main.py --config-file config/rtk_config.json
```

This allows you to:
- Persist settings across container updates
- Edit configuration files directly on the host
- Backup and restore configurations easily

## Requirements

- **BlueOS**: Core >= 1.1 (for BlueOS deployment)
- **GPS Hardware**: RTK-capable receiver (e.g., u-blox F9P based modules)
- **Network**: Internet connection for NTRIP caster access
- **Autopilot**: Compatible firmware (ArduPilot 4.1+ recommended)
- **Python**: 3.8+ with required dependencies

## Supported NTRIP Services

This extension works with any standard NTRIP caster, including:

- RTK2GO (free community service)
- Commercial NTRIP providers
- Private base stations
- CORS networks

## File Structure

```
app/
├── main.py              # Main application
├── pyproject.toml       # Python package configuration
├── config/              # Configuration directory
│   └── rtk_config.json  # Settings file (created automatically)
└── static/
    ├── index.html       # Web interface
    └── register_service # BlueOS service registration

requirements.txt         # Dependencies for local development
logs/                   # Log files (created automatically)
```

## Troubleshooting

### Connection Issues

If the NTRIP connection fails:

1. Verify your NTRIP caster URL and credentials
2. Check internet connectivity
3. Ensure the mountpoint is valid for your location
4. Review the error messages in the status section

### Local Development

If you encounter issues running locally:

1. Ensure all dependencies are installed: `pip install -r requirements.txt`
2. Check that the mavlink2rest service is running (if testing with real hardware)
3. Verify firewall settings aren't blocking the ports
4. Check the console output and log files in the `logs/` directory

### BlueOS Deployment

For BlueOS-specific issues:

1. Ensure the mavlink2rest URL is set to `http://host.docker.internal:6040`
2. Check that the extension has network access
3. Verify the autopilot is configured to accept GPS corrections

## Development

Built with:
- **Backend**: Python with Litestar framework
- **Frontend**: Modern HTML/CSS/JavaScript
- **Communication**: REST API with mavlink2rest integration
- **Storage**: Local JSON file for configuration
- **Logging**: Rotating file logs for debugging
