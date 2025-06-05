FROM python:3.11-slim

COPY app /app
RUN python -m pip install /app --extra-index-url https://www.piwheels.org/simple

EXPOSE 8000/tcp

LABEL version="0.0.3"

LABEL permissions='\
{\
  "ExposedPorts": {\
    "8000/tcp": {}\
  },\
  "HostConfig": {\
    "Binds":["/usr/blueos/extensions/ntrip-to-mavlink/config:/app/config"],\
    "ExtraHosts": ["host.docker.internal:host-gateway"],\
    "PortBindings": {\
      "8000/tcp": [\
        {\
          "HostPort": ""\
        }\
      ]\
    }\
  }\
}'

LABEL authors='[\
    {\
        "name": "Willian Galvani",\
        "email": "willian@bluerobotics.com"\
    }\
]'

LABEL company='{\
        "about": "",\
        "name": "Blue Robotics",\
        "email": "support@bluerobotics.com"\
    }'
LABEL type="other"
LABEL readme='https://raw.githubusercontent.com/williangalvani/BlueOS-Ntrip-extension/{tag}/README.md'
LABEL links='{\
        "source": "https://github.com/williangalvani/BlueOS-Ntrip-extension"\
    }'
LABEL requirements="core >= 1.1"

WORKDIR /app
ENTRYPOINT ["python", "main.py", "--host", "0.0.0.0", "--config-file", "config/rtk_config.json"]
