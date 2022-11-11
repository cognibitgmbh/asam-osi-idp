FROM python:3.9-slim
RUN apt-get update && apt-get install -y git unzip
ADD https://github.com/protocolbuffers/protobuf/releases/download/v21.9/protoc-21.9-linux-x86_64.zip /opt/protoc.zip
RUN unzip /opt/protoc.zip 'bin/*' 'include/*' -d /usr/local
RUN git config --global core.autocrlf true
RUN git clone -b 'v3.4.0' 'https://github.com/OpenSimulationInterface/open-simulation-interface.git' /opt/osi3
WORKDIR /opt/osi3
RUN pip install --upgrade pip
RUN pip install numpy autopep8 .
