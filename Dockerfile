# Vitis AI 3.5 host tools (quantizer, compiler) for the KV260 B4096 DPU.
# CPU image. Check the tag on Docker Hub (xilinx/vitis-ai-pytorch-cpu) before the first build.
#
#   docker build -t kv260-uav-vision-host .
#   docker run --rm -it -v "${PWD}:/workspace" kv260-uav-vision-host   (PowerShell)
#   docker run --rm -it -v "%cd%:/workspace" kv260-uav-vision-host     (cmd)
#
# Inside the container: conda activate vitis-ai-pytorch
ARG VITIS_AI_IMAGE=xilinx/vitis-ai-pytorch-cpu:ubuntu2004-3.5.0.306
FROM ${VITIS_AI_IMAGE}

WORKDIR /workspace
COPY pyproject.toml ./
COPY fc_link ./fc_link
COPY ingest ./ingest
COPY perception ./perception

# Install the project into the Vitis AI conda env so pytest and the code are available.
RUN bash -lc "source /opt/vitis_ai/conda/etc/profile.d/conda.sh \
    && conda activate vitis-ai-pytorch \
    && pip install --no-cache-dir -e '.[dev]'"

CMD ["bash", "-l"]
