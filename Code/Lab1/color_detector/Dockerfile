FROM duckietown/dt-duckiebot-interface:daffy-arm64v8

WORKDIR /color_detector_dir

COPY requirements.txt ./

RUN pip install -r requirements.txt

# Environment Variables
# Number of horizontal splits for each image
ENV "N_SPLITS" 1

# Number of bins to bin RGB values into
ENV "BINS" 10

# Maximum height of histogram bars
ENV "MAX" 5

COPY color_detector.py .

CMD python3 ./color_detector.py
