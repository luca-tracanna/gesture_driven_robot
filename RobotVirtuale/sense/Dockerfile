FROM python:3.10-slim

COPY *.py .
COPY requirements.txt .
RUN pip install -r requirements.txt

EXPOSE 8080
CMD ["python", "sense.py"]
