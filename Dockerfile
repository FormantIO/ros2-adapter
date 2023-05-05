FROM ros:humble-ros-base-jammy

RUN apt-get update
RUN apt-get install -y apt-utils
RUN apt-get install -y python3-dev python3-pip libffi-dev libgl1-mesa-glx


COPY ./requirements.txt .
RUN pip3 install -r requirements.txt

COPY . /app

ENTRYPOINT ["/app/start_docker.sh"]
CMD ["bash"]