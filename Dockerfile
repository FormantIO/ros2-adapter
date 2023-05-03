FROM ros:humble-ros-base-jammy

RUN apt-get update
RUN apt-get install -y apt-utils 
RUN apt-get install -y python3-dev python3-pip libffi-dev python3-opencv libgl1-mesa-glx libgl1
RUN apt-get update && apt-get install ffmpeg libsm6 libxext6  -y
RUN apt-get install -y ros-humble-cv-bridge



COPY ./requirements.txt .
RUN pip3 install -r requirements.txt

COPY . /app

ENTRYPOINT ["/app/start_docker.sh"]
CMD ["bash"]
