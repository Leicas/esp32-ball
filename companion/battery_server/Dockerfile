FROM python:3.12-alpine
WORKDIR /app
COPY server.py index.html ./
RUN mkdir /data && chown 10001:10001 /data
USER 10001:10001
CMD ["python", "-u", "server.py"]
