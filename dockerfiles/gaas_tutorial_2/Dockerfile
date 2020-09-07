FROM ct2034/vnc-ros-kinetic-full:latest

MAINTAINER Haoran 'doodleshr@giai.tech'

# add USER gi
RUN useradd --create-home --shell=/bin/bash gi

# COPY wallpaper image to container and set it as wallpaper
COPY	GAAS.png /root/gi/
COPY	models /usr/share/gazebo-7/models

# change to chinese APT sources
RUN	rm /etc/apt/sources.list \
	&& rm /usr/share/doro-lxde-wallpapers/kinetic.png \
	&& mv /root/gi/GAAS.png /usr/share/doro-lxde-wallpapers/kinetic.png

COPY	sources.list /etc/apt/


RUN	add-apt-repository -r ppa:fossfreedom/arc-gtk-theme-daily \
	&& apt-get -y update && apt-get install -y \
	vim \
	git \
	zip \
	qtcreator \
	cmake \
    	build-essential \
	genromfs \
	ninja-build \
	exiftool \
	python-argparse \
	python-empy \
	python-toml \
	python-numpy \
	python-yaml \
	python-dev \
	python-pip \
	ninja-build \
	protobuf-compiler \
	libeigen3-dev \
	libopencv-dev \
	python2.7 \
	python-pip \
	python-rosinstall \
	ros-kinetic-gazebo-* \
	&& pip install \
	pandas \
	jinja2 \
	pyserial \
	cerberus \
	pyulog \
	numpy \
	toml \
	pyquaternion


# copy px4 Firmware, Gazebo models and GAAS
RUN mkdir -p /root/gi/px4 \
	     /root/gi/GAAS \
	     /root/gi/catkin_ws/src \
	     /usr/share/gazebo-7/models


# clone MAVROS and build
RUN	apt-get -y update && apt-get install -y \
	python-catkin-tools \
	python-rosinstall-generator \
	ros-kinetic-mavros \
	ros-kinetic-mavros-extras \
	wget \
	&& apt list --installed ros-kinetic* | grep upgradable | sed s#/.*## | xargs sudo apt install -y \
	cd /root/gi/px4 \
	&& wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh \
	&& chmod 777 /root/gi/px4/install_geographiclib_datasets.sh && /root/gi/px4/install_geographiclib_datasets.sh


# px4 Firmware clone and make
RUN cd ~/root/gi/px4/ \
	&& git clone https://github.com/PX4/Firmware.git \
	&& cd ~/root/gi/px4/Firmware \
        && git checkout v1.8.0 \
	&& make posix_sitl_default


# update environment variables

ENV ros_package_path '$ROS_PACKAGE_PATH'
ENV gazebo_model_path '$GAZEBO_MODEL_PATH'

RUN echo "export GAZEBO_MODEL_PATH='${gazebo_model_path}':/usr/share/gazebo-7/models/ \n" >> /root/.bashrc \
	&& echo "source /root/gi/px4/Firmware/Tools/setup_gazebo.bash /root/gi/px4/Firmware/ /root/gi/px4/Firmware/build/posix_sitl_default" >> /root/.bashrc \
	&& echo 'export ROS_PACKAGE_PATH='${ros_package_path}':/root/gi/px4/Firmware/' >> /root/.bashrc \
	&& echo 'export ROS_PACKAGE_PATH='${ros_package_path}':/root/gi/px4/Firmware/Tools/sitl_gazebo' >> /root/.bashrc
