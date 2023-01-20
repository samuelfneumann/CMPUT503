FROM duckietown/dt-duckiebot-interface:daffy-arm64v8

WORKDIR /color_detector_dir

COPY requirements.txt ./

RUN pip install -r requirements.txt
# RUN cat requirements-apt.txt | xargs sudo apt install -y
RUN apt list --installed
ENV "N_SPLITS" 1
ENV "BINS" 10
ENV "MAX" 5

COPY color_detector.py .

CMD python3 ./color_detector.py
