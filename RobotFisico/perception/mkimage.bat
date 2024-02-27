docker stop perc_module
docker rm perc_module
docker rmi perc_image
docker build -t app/perc_image .
docker run -d --name perc_module -p 8100:8100 app/perc_image
docker ps
