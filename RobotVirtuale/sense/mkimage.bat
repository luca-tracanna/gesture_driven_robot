docker stop sense_module
docker rm sense_module
docker rmi sense_image
docker build -t app/sense_image .
docker run -d --name sense_module -p 8080:8080 app/sense_image
docker ps
