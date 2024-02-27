docker stop action_module
docker rm action_module
docker rmi action_image
docker build -t app/action_image .

