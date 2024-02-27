docker stop contr_module
docker rm contr_module
docker rmi contr_image
docker build -t app/contr_image .

