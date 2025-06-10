FROM python:3.12-slim-bullseye

ADD https://github.com/krallin/tini/releases/download/v0.19.0/tini /tini
RUN chmod +x /tini

COPY requirements.txt requirements.txt
RUN pip3 install --no-cache-dir -r requirements.txt

COPY main.py /main.py

ENTRYPOINT ["/tini", "-g", "--", "python", "main.py"]